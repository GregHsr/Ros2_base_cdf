#!/usr/bin/env python
# coding=utf-8
# license removed for brevity
import rclpy
from rclpy.node import Node
from cdf_msgs.msg import PicAction
from std_msgs.msg import String
from os import walk
import sys
from std_srvs.srv import Trigger
import serial
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
SERIAL_SEND_CHUNK_SIZE = 100

robot_cards_cfg = {
    "USER": ["/dev/ttyAMA0", 115200, 'hardware'],
    "CAN": ["/dev/ttyUsbCANCard", 115200, 'version_request'],
    "SERVO": ["/dev/ttyUsbServoCard", 115200, 'version_request']
}

robot_cards_cfg = {
    "USER": ["/dev/ttyAMA0", 115200, 'hardware'],
    "CAN": ["/dev/ttyUsbCANCard", 115200, 'version_request'],
    "SERVO": ["/dev/ttyUsbCANCard", 115200, 'version_request']
}


class UartCom(Node):
    def __init__(self):
        super().__init__('uart_com_node')

        self.default_exec = MultiThreadedExecutor()
        self.default_callback_gr = ReentrantCallbackGroup()
        # Set relative topic path
        self.action_topic = "action"
        self.pic_message_inject_topic = "debug/pic_message_inject"
        self.raw_received_pic_msg_topic = "debug/raw_received_pic_msg"

        # Declare parameters
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('config_file', "default_cdf_2023")
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('pic_msg_freq', 10.0)

        # Gettting parameters
        self.debug_mode = self.get_parameter('debug_mode') \
                              .get_parameter_value() \
                              .bool_value
        self.config_file = self.get_parameter('config_file') \
                               .get_parameter_value() \
                               .string_value
        self.simulation_mode = self.get_parameter('simulation_mode') \
                                   .get_parameter_value() \
                                   .bool_value

        # frequence 9Hz alors que pic est a 10Hz => on est sur de rater des
        # trames mais il est quasiment impossible de rater plusieurs trames
        # d'affilé
        self.freq = self.get_parameter('pic_msg_freq') \
                        .get_parameter_value() \
                        .double_value

        # In debug mode, we just spin for an infinite time
        if self.debug_mode:
            self.get_logger().info("debug mode activated")
            rclpy.spin(self, self.default_exec)
            return

        # Declare the health service
        self.health = " ".join(robot_cards_cfg.keys())
        self.healthsrv = self.create_service(Trigger,
                                             'health',
                                             self.health_handler,
                                             callback_group=self.default_callback_gr
                                             )

        # On charge le bon dictionnaire de message en fonction du fichier
        # de configuration spécifié
        try:
            exec(
                f"from .resources.{self.config_file} import msgs_dict",
                globals()
            )

            self.msgs_dict = msgs_dict
        except ModuleNotFoundError:
            self.get_logger().fatal("Config file not found : %s"
                                    % self.config_file)
            sys.exit(1)
        except ImportError:
            self.get_logger().fatal("'msgs_dict' variable not found in"
                                    "config file : %s" % self.config_file
                                    )
            sys.exit(1)
        except Exception as e:
            self.get_logger().fatal("Unexpected error while loading"
                                    "config file : \n%s" % e
                                    )

        # declaration des publishers
        # on prend msg_format et on ajoute a chaque element un champ pub et un
        # champ old_message
        for msg_dict_key, msg_dict_value in self.msgs_dict.items():
            for msglabel, value in msg_dict_value.items():
                publisher = self.create_custom_publisher(**value)
                msg_dict_value[msglabel]["pub"] = publisher
                old_message = None
                for key, element in value["structure"].items():
                    msg_dict_value[msglabel]["old_message"] = old_message

        # Initializing the node depanding on its options

        if self.simulation_mode:
            self.create_subscription(String,
                                     self.pic_message_inject_topic,
                                     lambda msg: self.parse_bytes(msg.data),
                                     10)
        else:
            self.cardsHandlers = dict()
            self.last_end_data = dict()

            for card_name in robot_cards_cfg.keys():
                self.last_end_data[card_name] = ''

            # Initialize the main rate to flush the serial buffer
            self.main_timer = self.create_timer(2/self.freq, lambda:
                                                self.run(self.cardsHandlers),
                                                callback_group=self.default_callback_gr
                                                )

            self.create_subscription(
                PicAction,
                self.action_topic,
                self.action_topic_callback,
                callback_group=self.default_callback_gr,
                qos_profile=100
            )

            self.connectCards(robot_cards_cfg)

            self.get_logger().info("All of the Robot's cards connected :)")

        self.health = "HEALTHY"

        try:
            rclpy.spin(self)
        except Exception as e:
            self.get_logger().error("Error :", e)
        except RuntimeError as e:
            self.get_logger().error(e)

    def health_handler(self, request, response):
        if self.health == "HEALTHY":
            response.success = True
        else:
            response.success = False

        response.message = self.health
        return response

    def run(self, cardsHandler):
        try:
            # On recupere les trames de tous les ports successivement
            # afin de pouvoir les parser et ensuite les publier si
            # elles sont valides
            for card_name, handler in cardsHandler.items():
                if handler is None:
                    continue

                char_available = handler.in_waiting
                if char_available != 0:
                    received_data = self.last_end_data[card_name] \
                        + str(handler.read(char_available).decode('UTF8'))
                    try:
                        data_sequences = received_data.split('\n')
                    except Exception:
                        self.get_logger().info("Error While Decoding Data : %s"
                                               % received_data[1])
                        data_sequences = []
                    self.last_end_data[card_name] = data_sequences.pop()
                    for data_sequence in data_sequences:
                        self.parse_bytes(data_sequence)

        except Exception as e:
            self.get_logger().error("Unhandled Exception : %s" % e)

    def create_custom_publisher(self, topic_path,
                                type, *args, **kwargs):
        """
        Cree un publisher a partir d'un rosparam
        :param topic_path: path relatif du topic
        :param type: le type de message
        :param args: non utilisé ( mais ne pas enlever pour autant ! )
        :param kwargs: non utilisé ( mais ne pas enlever pour autant ! )
        :return: le publisher
        """
        if topic_path[0] == "/":
            self.get_logger().warn(
                r"The following topic {} will be referenced as global topic"
                r" and will therefore ignore the namespace specified during"
                r" node initialization".format(topic_path)
            )
        return self.create_publisher(type, topic_path, 10,
                                     callback_group=self.default_callback_gr)

    def publish_value(self, msg, pub, publish_on_change=True,
                      old_message=None):
        """
        Factorise le code d'envoie d'un message
        :param value: valeur a envoyer, doit etre du bon type!
        :param msg: message object, deja instancie (ex Int16() est un
                    param valide
        :param pub: le publisher
        :param publish_on_change: si True, publie le message
               uniquement si la valeur a change depuis le dernier envoi
        :param old_message: ancienne valeur, utile si publish_on_change=True
        :return: la nouvelle valeur
        """
        self.get_logger().debug("%s => %s" % (old_message, msg))
        if (not publish_on_change) or (msg != old_message):
            pub.publish(msg)
        return msg

    def parse_bytes(self, read_bytes):
        # On sépare la trame en morceaux séparés par une virgule
        parsed_values = read_bytes.split(',')
        try:
            # On récupère le type/nom de la trame recue
            selected_dict = self.msgs_dict[parsed_values.pop(0)]
            # On calcule le nombre d'éléments attendus dans la trame recue
            msg_lenght = 0
            for element in selected_dict.values():
                msg_lenght += len(element['structure'])

            # On vérifie que le nombre d'éléments présent dans la tramme
            # est correct
            if (len(parsed_values) == msg_lenght):
                self.get_logger().debug('succesfully parsed %s' % read_bytes)

                # On parcourt la structure de la trame recue pour interpréter
                # correctement les valeurs
                for msglabel, value in selected_dict.items():
                    # Contruction du message
                    msg = value["type"]()
                    for key, element in value["structure"].items():
                        path = element["path"]
                        element_value = element["value_expression"](
                            parsed_values[key]
                        )
                        exec("msg.%s = %s" % (path, element_value))

                    # On publie le message
                    selected_dict[msglabel]["old_message"] = \
                        self.publish_value(
                            msg=msg,
                            pub=value["pub"],
                            publish_on_change=value["publish_on_change"],
                            old_message=selected_dict[msglabel]["old_message"]
                        )
            else:
                # Si le nombre d'éléments n'est pas correct, on loggue l'erreur
                self.get_logger().warn("'%s' was not parsed because the frame"
                                       "takes %d values (%d given)"
                                       % (read_bytes,
                                          msg_lenght,
                                          len(parsed_values))
                                       )
        except KeyError as e:
            self.get_logger().warn("The following key doesn't exist "
                                   "in the msgs_dict: %s" % e)

    def action_topic_callback(self, msg):
        """
        Callback function that writes a specified message to a specified card.

        Parameters
        ----------
        msg : string
            The message to write to the card.
        """
        start_time = time.time()
        try:
            self.send_data(self.cardsHandlers[msg.action_destination],
                           msg.action_msg)
        except KeyError:
            self.get_logger().error("requested serial port destination (%s) \
                                     not found" % msg.action_destination)
        print("exec time : %f" % (time.time() - start_time))

    def send_data(self, serial_port, data):
        if False:
            # On découpe la chaine de caractère a envoyer en paquets de
            # 100 caractères

            # C'est surtout pour envoyer des très longue chaine comme dans
            # le cas de du bootloader

            chunks = [data[i:i+SERIAL_SEND_CHUNK_SIZE]
                      for i in range(0, len(data), SERIAL_SEND_CHUNK_SIZE)
                      ]
            for chunk in chunks:
                serial_port.write(chunk.encode('utf-8'))
            serial_port.write('\n'.encode('utf-8'))
        else:
            serial_port.write((data + '\n').encode('utf-8'))

    def hardwareCon(self, card_name, card_config):
        while True:
            try:
                card_serial = serial.Serial(card_config[0],
                                            card_config[1])

                self.get_logger().info(
                    "Connected to card '%s'"
                    % card_name
                )
                self.health = self.health.replace(card_name, "")
                return card_serial
            except serial.SerialException:
                self.get_logger().warn("Can't open Hardware serial ports !")
            except Exception as e:
                self.get_logger().error("Unhandled exception occured :\n", e)

            self.get_logger().warn(
                "Retrying to connect the '%s' card in 1s"
                % card_name
            )
            rclpy.spin_once(self, timeout_sec=1, executor=self.default_exec)

    def versionBasedCon(self, card_name, card_config):
        while True:
            filenames = next(walk("/dev"), (None, None, []))[2]
            valid_filenames = []
            for filename in filenames:
                if filename.startswith(("CartePic")):
                    valid_filenames.append(filename)
            for filename in valid_filenames:
                self.get_logger().info("trying to connect to %s" % filename)
                try:
                    tested_serial = serial.Serial("/dev/" + filename,
                                                  card_config[1],
                                                  timeout=0,
                                                  write_timeout=0)
                    tested_serial.write("VERSION\n".encode("utf-8"))

                    # Wait for 0.2 seconds and try to found the VERSION keyword
                    # in the input buffer
                    rclpy.spin_once(self, timeout_sec=0.2, executor=self.default_exec)

                    all_data = tested_serial.read(tested_serial.in_waiting).decode('utf-8')
                    for line in all_data.split('\n'):
                        print(line)
                        splited = line.split(',')
                        if splited[0] == "version" and splited[1] == card_name:
                            self.health = self.health.replace(card_name, "")
                            return tested_serial
                        else:
                            raise ValueError("unknown card type")
                except serial.SerialException as e:
                    self.get_logger().error(
                        "Scanned serial port is already opened nor existing !")
                    print(e)
                except ValueError:
                    self.get_logger().info("Unknown card found")
                    tested_serial.close()
            self.get_logger().warn(
                "Retrying to connect the '%s' card in 1s"
                % card_name
            )
            rclpy.spin_once(self, timeout_sec=1, executor=self.default_exec)

    def connectCards(self, cards_config):
        """
        Connect the cards specified in the card_config configuration
        dictionnary

        Parameters
        ----------
        cards_config : dict()
            Contains the configuration of all the card of the robot

        Returns
        -------
        cards : dict()
            Contains all the newly created card's handler
        """
        for key, value in cards_config.items():
            card_handler = None

            while card_handler is None:
                if value[2] == "hardware":
                    card_handler = self.hardwareCon(key, value)
                elif value[2] == "version_request":
                    card_handler = self.versionBasedCon(key, value)
                else:
                    raise ValueError("unknown card type")

            self.cardsHandlers[key] = card_handler


def main():
    rclpy.init(args=None)

    UartCom()


if __name__ == '__main__':
    main()

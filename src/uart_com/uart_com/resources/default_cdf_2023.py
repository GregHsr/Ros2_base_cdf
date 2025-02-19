from cdf_msgs.msg import RobotData, ServoDebug, Buttons, AxResult
from std_msgs.msg import String, Bool, Int16

msgs_dict = {
    "motor": {
        "robot_data": {
            "topic_path": "Odom",
            "type": RobotData,
            "publish_on_change": False,
            "structure": {
                0: {
                    "value_expression": lambda x: 1,
                    "path": "pic_time.data"
                },
                2: {
                    "value_expression": lambda x: float(x),
                    "path": "position.x"
                },
                3: {
                    "value_expression": lambda x: float(x),
                    "path": "position.y"
                },
                4: {
                    "value_expression": lambda x: float(x),
                    "path": "theta.data"
                },
                5: {
                    "value_expression": lambda x: float(x),
                    "path": "vx.data"
                },
                6: {
                    "value_expression": lambda x: float(x),
                    "path": "vy.data"
                },
                7: {
                    "value_expression": lambda x: int(x),
                    "path": "pwm_left.data"
                },
                8: {
                    "value_expression": lambda x: int(x),
                    "path": "pwm_right.data"
                }
            }
        },
        "motion_done": {
            "topic_path": "motion_done",
            "type": Bool,
            "publish_on_change": False,
            "structure": {
                1: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "data"
                }
            }
        }
    },
    "version": {
        "pic_version": {
            "topic_path": "init/version_PIC",
            "type": String,
            "publish_on_change": False,
            "structure": {
                0: {
                    "value_expression": lambda x: "'" + x + "'",
                    "path": "data"
                }
            }
        }
    },
    "action_result": {
        "action_result": {
            "topic_path": "ax12_result",
            "type": AxResult,
            "publish_on_change": False,
            "structure": {
                0: {
                    "value_expression": lambda x: "'" + x + "'",
                    "path": "action_name"
                },
                1: {
                    "value_expression": lambda x: int(x),
                    "path": "result"
                }
            }
        }
    },
    "AX12DEBUG": {
        "AX12DEBUG": {
            "topic_path": "servo_debug",
            "type": ServoDebug,
            "publish_on_change": False,
            "structure": {
                0: {
                    "value_expression": lambda x: int(x),
                    "path": "ID"
                },
                1: {
                    "value_expression": lambda x: int(x),
                    "path": "Present_Position"
                },
                2: {
                    "value_expression": lambda x: int(x),
                    "path": "Present_Speed"
                },
                3: {
                    "value_expression": lambda x: int(x),
                    "path": "Present_Load"
                },
                4: {
                    "value_expression": lambda x: int(x),
                    "path": "Present_Voltage"
                },
                5: {
                    "value_expression": lambda x: int(x),
                    "path": "Present_Temperature"
                },
                6: {
                    "value_expression": lambda x: int(x),
                    "path": "Moving"
                }
            }
        }
    },
    "AX12SCANDONE": {
        "AX12SCANDONE": {
            "topic_path": "AX12SCANDONE",
            "type": Int16,
            "publish_on_change": False,
            "structure": {
                0: {
                    "value_expression": lambda x: int(x),
                    "path": "data"
                }
            }
        }
    },
    "AX12UNREACHABLE": {
        "AX12UNREACHABLE": {
            "topic_path": "AX12UNREACHABLE",
            "type": Int16,
            "publish_on_change": False,
            "structure": {
                0: {
                    "value_expression": lambda x: int(x),
                    "path": "data"
                }
            }
        }
    },
    "bootloader": {
        "bootloader": {
            "topic_path": "Bootloader_complete",
            "type": String,
            "publish_on_change": False,
            "structure": {
                0: {
                    "value_expression": lambda x: "'" + x + "'",
                    "path": "data"
                }
            }
        }
    },
    "buttons": {
        "buttons": {
            "topic_path": "buttons",
            "type": Buttons,
            "publish_on_change": False,
            "structure": {
                0: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "emmergency_stop.data"
                },
                1: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "leach.data"
                },
                2: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "big_sw.data"
                },
                3: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "up.data"
                },
                4: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "down.data"
                },
                5: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "enter.data"
                },
                6: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "little_sw_1.data"
                },
                7: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "little_sw_2.data"
                },
                8: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "little_sw_3.data"
                },
                9: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "little_sw_4.data"
                },
                10: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "little_sw_5.data"
                },
                11: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "little_sw_6.data"
                },
                12: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "little_sw_7.data"
                },
                13: {
                    "value_expression": lambda x: bool(int(x)),
                    "path": "little_sw_8.data"
                }
            }
        }
    },
    "Pos": {
        "Pos": {
            "topic_path": "pos_done",
            "type": Bool,
            "publish_on_change": False,
            "structure": {
                0: {
                    "value_expression": lambda x: (x == 'done'),
                    "path": "data"
                }
            }
        }
    }
}

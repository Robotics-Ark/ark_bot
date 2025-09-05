driver = STSArmDriver(
    component_name="sts_arm",
    component_config={
        "real_config": {
            "port": "COM5",
            "baudrate": 1000000,
            "motor_ids": [1,2,3,4,5,6,7],
            "joint_order": ["j1","j2","j3","j4","j5","j6","j7"],
            "gear_ratios": {"1": 1.0,"2": 3.0,"3": 3.0,"4": 3.0,"5": 3.0,"6": 3.0},
            "zero_offsets_deg": {"1": 0.0, "2": 0.0, "3": 0.0, "4": 0.0, "5": 0.0, "6": 0.0},
            "ticks_per_turn": 4096,
            "speed_default": 1200,
            "acc_default": 50,
            "gripper": {
                "name": "gripper_width",
                "id": 9,
                "min_width_m": 0.0,
                "max_width_m": 0.08,
                "min_ticks": 1000,
                "max_ticks": 3000,
                "speed": 1000,
                "acc": 30
            }
        }
    },
    sim=False
)

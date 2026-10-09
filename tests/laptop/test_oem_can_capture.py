from laptop.diagnostics.capture_oem_can import decode_gripper


def test_decode_gripper_sdk_bytes():
    decoded=decode_gripper({'can_id_raw':0x159,'dlc':8,'data_hex':'0001117000640100'})
    assert decoded==dict(target_raw=70000,nominal_opening_mm=70,effort_raw=100,
                        nominal_effort_nm=0.1,control_code=1,set_zero_code=0)


def test_feedback_and_short_frame_not_command():
    assert decode_gripper({'can_id_raw':0x2a8,'dlc':8}) is None
    assert decode_gripper({'can_id_raw':0x159,'dlc':4}) is None

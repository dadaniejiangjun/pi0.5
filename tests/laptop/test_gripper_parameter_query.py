from laptop.diagnostics.query_piper_gripper_parameters import (
    QUERY_CAN_ID,
    QUERY_PAYLOAD,
    REQUIRED_FEEDBACK_IDS,
    RESPONSE_CAN_ID,
    decode_gripper_parameter_payload,
    dispatch_query_if_feedback_fresh,
    evaluate_feedback_preflight,
)


def _fresh_feedback(now_ns: int):
    counts = {can_id: 2 for can_id in REQUIRED_FEEDBACK_IDS}
    timestamps = {can_id: now_ns - 10_000_000 for can_id in REQUIRED_FEEDBACK_IDS}
    return counts, timestamps


def test_query_contract_is_one_read_only_gripper_parameter_request():
    assert QUERY_CAN_ID == 0x477
    assert RESPONSE_CAN_ID == 0x47E
    assert QUERY_PAYLOAD == bytes((0x04, 0, 0, 0, 0, 0, 0, 0))


def test_decode_documented_gripper_limit_fields_and_raw_payload():
    result = decode_gripper_parameter_payload(bytes((150, 70, 3, 0, 0, 0, 0, 0)))
    assert result["teaching_range_percent"] == 150
    assert result["teaching_range_percent_documented_valid"] is True
    assert result["max_range_config_mm"] == 70
    assert result["max_range_config_documented_valid"] is True
    assert result["teaching_friction"] == 3
    assert result["raw_payload_hex"] == "9646030000000000"


def test_decode_preserves_unrecognized_maximum_without_claiming_validity():
    result = decode_gripper_parameter_payload(bytes((0, 42, 0, 0, 0, 0, 0, 0)))
    assert result["max_range_config_mm"] == 42
    assert result["max_range_config_documented_valid"] is False


def test_decode_rejects_wrong_width():
    try:
        decode_gripper_parameter_payload(b"\x00" * 7)
    except ValueError as exc:
        assert "exactly 8 bytes" in str(exc)
    else:
        raise AssertionError("malformed payload was accepted")


def test_preflight_requires_all_five_fresh_decoded_feedback_ids():
    now_ns = 5_000_000_000
    counts, timestamps = _fresh_feedback(now_ns)
    result = evaluate_feedback_preflight(counts, timestamps, now_ns=now_ns)
    assert result["ready"] is True
    assert result["missing_ids"] == []
    assert result["stale_ids"] == []


def test_preflight_rejects_missing_stale_and_future_feedback():
    now_ns = 5_000_000_000
    counts, timestamps = _fresh_feedback(now_ns)
    missing_id = min(REQUIRED_FEEDBACK_IDS)
    counts.pop(missing_id)
    timestamps[0x2A8] = now_ns - 251_000_000
    timestamps[0x2A7] = now_ns + 1
    result = evaluate_feedback_preflight(counts, timestamps, now_ns=now_ns)
    assert result["ready"] is False
    assert f"0x{missing_id:03X}" in result["missing_ids"]
    assert "0x2A8" in result["stale_ids"]
    assert "0x2A7" in result["future_or_invalid_ids"]


def test_dispatch_does_not_send_when_feedback_preflight_fails():
    now_ns = 5_000_000_000
    sent = []
    result = dispatch_query_if_feedback_fresh(
        lambda can_id, payload: sent.append((can_id, payload)),
        {},
        {},
        now_ns=now_ns,
    )
    assert result["sent"] is False
    assert sent == []


def test_dispatch_sends_exactly_one_whitelisted_request_after_fresh_preflight():
    now_ns = 5_000_000_000
    counts, timestamps = _fresh_feedback(now_ns)
    sent = []
    result = dispatch_query_if_feedback_fresh(
        lambda can_id, payload: sent.append((can_id, payload)),
        counts,
        timestamps,
        now_ns=now_ns,
    )
    assert result["sent"] is True
    assert sent == [(0x477, bytes((0x04, 0, 0, 0, 0, 0, 0, 0)))]

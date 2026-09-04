from stage3_red_pad_alignment import red_station_right_offset_m


def test_red_station_is_right_of_black_base_reference():
    assert red_station_right_offset_m(19.5) == 0.195

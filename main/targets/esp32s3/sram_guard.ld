ASSERT(_iram_end <= 0x403c0000, "S3 RF dump overlaps IRAM")
ASSERT(_data_end <= 0x3fcd0000, "S3 RF dump overlaps data")
ASSERT(_bss_end <= 0x3fcd0000, "S3 RF dump overlaps BSS")

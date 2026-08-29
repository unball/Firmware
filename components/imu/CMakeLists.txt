idf_component_register(
    SRCS "imu.cpp"
    INCLUDE_DIRS "." "../../include"
    REQUIRES
        mpu6050
        i2cdev
)
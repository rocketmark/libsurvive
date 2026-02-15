
add_executable(test_pose_roundtrip
    test_pose_roundtrip.c
)

target_link_libraries(test_pose_roundtrip
    survive
    m
)

add_test(NAME pose_roundtrip COMMAND test_pose_roundtrip)


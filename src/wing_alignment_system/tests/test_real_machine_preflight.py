from wing_alignment_system.real_machine_preflight import (
    check_huatai_one_processes,
    check_memory,
    check_ros_duplicate_nodes,
    duplicate_names,
)


def test_memory_check_fails_when_swap_is_already_heavy():
    result = check_memory(
        {
            "MemAvailable": 2 * 1024 * 1024,
            "SwapTotal": 2 * 1024 * 1024,
            "SwapFree": 256 * 1024,
        },
        min_mem_available_mib=1536.0,
        max_swap_used_mib=512.0,
    )

    assert result.status == "fail"
    assert "swap" in result.message
    assert result.details["swap_used_mib"] == 1792.0


def test_huatai_one_check_fails_on_duplicate_local_processes():
    rows = [
        {
            "pid": 10,
            "ppid": 9,
            "rss_kib": 100000,
            "pcpu": 1.0,
            "args": "/home/ls/ros_motor_qt_new/install/huatai_one/lib/huatai_one/huatai_one",
        },
        {
            "pid": 20,
            "ppid": 19,
            "rss_kib": 110000,
            "pcpu": 2.0,
            "args": "/home/ls/ros_motor_qt_new/install/huatai_one/lib/huatai_one/huatai_one",
        },
        {
            "pid": 19,
            "ppid": 1,
            "rss_kib": 20000,
            "pcpu": 0.0,
            "args": "/usr/bin/python3 /opt/ros/humble/bin/ros2 run huatai_one huatai_one",
        },
    ]

    result = check_huatai_one_processes(rows, max_huatai_one=1)

    assert result.status == "fail"
    assert result.details["count"] == 2


def test_duplicate_ros_names_ignore_warnings():
    assert duplicate_names(
        [
            "/motor_control_qt",
            "WARNING: duplicate names",
            "/motor_control_qt",
            "/formation_controller",
        ]
    ) == {"/motor_control_qt": 2}


def test_ros_duplicate_node_check_fails_on_duplicate_names():
    def fake_run(cmd, timeout_sec):
        assert "node list" in " ".join(cmd)
        return "/motor_control_qt\n/motor_control_qt\n"

    result = check_ros_duplicate_nodes(skip=False, run=fake_run)

    assert result.status == "fail"
    assert result.details["duplicates"] == {"/motor_control_qt": 2}

namespace rmcs_msgs {
enum class DartLaunchStages {
    DISABLE,
    INIT,
    LOADING,
    RESETTING,
    READY,
    CANCEL,
};
/*
说明：
    DISABLE：电机全部失能，遥控器双下
    INIT：初始状态，需要滑台复位
    LOADING：上膛和填装中
    RESETTING: 同步带复位中
    READY：可发射状态。发射完成后进入INIT
特殊：
    READY -> INIT：退膛，转动带下去接滑台使其复位，避免空放
*/
} // namespace rmcs_msgs
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
enum class DartFillingStages {
    INIT,
    DOWNING,
    READY,
    EMPTY,
    LIFTING,
    FILLING,
};
/*
说明：
    INIT：初始状态，已经有一发镖在升降平台上，也代表着升降平台在up位置且有镖的状态
    DROWING：升降平台下降
    READY：下降到位，镖被推上滑台，可以发射，双READY发射
    EMPTY：表示升降平台无镖，调试用，不参与控制
    LIFTING：升降平台上升
    FILLING：正在填装中，填装完成进入INIT 
*/
} // namespace rmcs_msgs
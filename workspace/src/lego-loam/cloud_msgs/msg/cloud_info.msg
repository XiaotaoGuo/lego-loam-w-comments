Header header 

int32[] startRingIndex      # 每一行激光点在数组中的起始索引
int32[] endRingIndex        # 每一行激光点在数组中的末尾索引

float32 startOrientation    # 第一束激光的朝向
float32 endOrientation      # 最后一束激光的朝向
float32 orientationDiff     # 激光束转过角度

bool[]    segmentedCloudGroundFlag # true - ground point, false - other points
uint32[]  segmentedCloudColInd # point column index in range image
float32[] segmentedCloudRange # point range 
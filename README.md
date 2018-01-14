# NHK2018_Odometry
odometry node for stm32f103

rosserialを通してROSネットワークにオドメトリデータを提供するノードです．

2つのロータリエンコーダと1つのIMU(InvenSense製 MPU-9250)のデータを内部で積分し，位置と姿勢を*geometry_msgs::Pose*のトピックとしてpublishします．

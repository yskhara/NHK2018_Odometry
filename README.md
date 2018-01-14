# NHK2018_Odometry
odometry node for stm32f103

rosserialを通してROSネットワークにオドメトリデータを提供するノードです．

2つのロータリエンコーダと1つのIMU(InvenSense製 MPU-9250)のデータを，*geometry_msgs::TwistWithCovarianceStamped*と*sensor_msgs::Imu*のトピックとしてpublishします．
これらのトピックは，例えば，ekf\_localizationノードに直接与えることができます．

# 無人機自主追蹤並降落在移動的無人地面載具上

目錄：
- 實驗平台
- 系統架構
- 視覺偵測
- 控制器設計
- 真實硬體飛行結果(成功、失敗、統計成功失敗次數)

## 1. 實驗平台

無人機(UAV)的追蹤與降落實驗平台：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure1.png)

- 無人機使用H420軸之Pixhawk 6C飛控小型無人機。包含F9P GPS、數傳模組以及電池。
- 無人機上安裝Jeston Orin NX，用於處理圖像、物件檢測和控制算法等，並安裝攝影機獲取影像資訊。
- 無人地面載具(UGV)速度設為0.2m/s。

實驗場景示意圖：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure2.png)

- UGV往西方方向並以0.2m/s速度進行直線移動。
- 無人機將採用自行設計控制算法，進行追蹤與降落，最終會降落在UGV上方。

座標轉換圖：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure3.png)

- 分為世界座標、無人機座標和攝影機座標。
- 世界座標與無人機座標是透過東西南北向，和無人機x,y軸位置來固定。
- 若要控制無人機讓影像中心點靠近H平台正中心，則給予無人機速度為： vx = C_cx - BB_cx, vy = BB_cy - C_cy。此公式將會在追蹤控制器中使用到。

## 2. 系統架構

完整追蹤與降落控制系統架構圖：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure4.png)

- 分為偏航校正、追蹤和降落控制模組。
- 偏航校正控制模組：旋轉無人機偏航角到所指定的航向，以對齊所設計之坐標系。
- 追蹤控制模組：用來控制無人機X, Y, Z速度的追蹤控制器，根據YOLO-tiny所檢測到的較大的H降落平台的中心點位置和面積大小做為輸入。
- 降落控制模組：用來控制無人機X, Y, Z速度的降落控制器，根據ArUco marker所檢測到的較小的marker的中心點位置和面積大小做為輸入。

整體系統架構圖：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure5.png)

- 邊緣運算器為Jetson Orin NX，軟體包含ROS系統、攝影機影像擷取、追蹤控制模組、降落控制模組和MAVROS。
- 無人機飛控版週邊包含GPS、IMU、數傳模組和Battery。
- 地面控制站主要遠端控制NX，並觀看影像畫面，以及無人機電池監控和遙控器緊急控制。

整體系統流程圖：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure6.png)

- 第一階段：起飛與偏航校正，無人機面朝北後進入下一階段。
- 第二階段：PD 追蹤控制器根據 YOLO-tiny 檢測的降落平台，使用中心點誤差和 FNN 預測高度控制無人機追蹤，當高度低於閾值後進入下一階段。
- 第三階段：模糊自適應 P 降落控制器，偵測不到 ArUco marker 時無人機上升，僅控制 (x, y) 方向速度，z 軸保持等速下降，當 ArUco marker 大小與位置符合降落條件時，啟動姿態與推力控制完成降落。
  
## 3. 視覺偵測
## 4. 控制器設計
## 5. 真實硬體飛行結果(成功、失敗、統計成功失敗次數)

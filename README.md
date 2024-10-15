# 無人機自主追蹤並降落在移動的無人地面載具上

目錄：
- 實驗平台
- 系統架構
- 視覺偵測
- 控制器設計
- 真實硬體飛行結果

## 1. 實驗平台

無人機(UAV)的追蹤與降落實驗平台：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure1.png)

- 無人機使用H420軸之Pixhawk 6C飛控小型無人機。包含F9P GPS、數傳模組以及電池。
- 無人機上安裝Jeston Orin NX，用於處理圖像、物件檢測和控制算法等，並安裝攝影機獲取影像資訊。
- 無人地面載具(UGV)速度設為0.2m/s。
- H降落平台大小為60cmx60cm，ArUco marker為6cm*6cm。

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

PD追蹤控制模組中的目標檢測方法：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure7.png)

- 使用YOLOv4-tiny檢測架構，將其實現在Jetson Orin NX上，並使用GPU加速推論，FPS達20左右。

PD追蹤控制模組中的高度預測方法：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure8.png)

- 所設計之FNN高度預測器，輸入為YOLOv4-tiny所檢測到H降落平台之pixel大小，輸出為所對應到無人機與H降落平台之間高度。

模糊自適應 P 降落控制器中的ArUco marker檢測方法：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure9.png)

- 使用ArUco marker檢測算法，以檢測小型ArUco marker。

無人機的不同偏航角度範圍定義圖：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure10.png)

- 當偏航角度範圍較大時，無人機可以快速旋轉；隨著偏航角接近 0°（朝北方），需要逐漸減速來抵消慣性。
- 當偏航角進入 Range1 區域時，應將偏航角速度降至零，使無人機穩定面向 0° 方位。

偏航校正控制器之流程圖：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure11.png)

- 根據無人機的不同偏航角度範圍定義來設計偏航角速度。

## 4. 控制器設計 here

PD追蹤控制器架構圖：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure12.png)

- 包含ROS、YOLOv4-tiny和FNN模型之整合，以控制無人機X, Y, Z軸速度控制進行追蹤。

模糊自適應P降落控制器架構圖：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure13.png)

- 包含ROS和ArUco marker整合，以控制無人機X, Y軸速度控制進行降落，Z軸速度為等速下降。

X 軸速度之模糊自適應 P 控制器之隸屬函數設計：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure19.png)

- 𝑒𝑝𝑜𝑠_𝑥範圍定義在-320~320 之間、𝑑𝑒𝑝𝑜𝑠_𝑥範圍定義-80~80 之間。𝐾𝑓𝑝_𝑥(輸出)範圍範圍定義-0.002~0.002 之間。

Y 軸速度之模糊自適應 P 控制器之隸屬函數設計：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure20.png)

- 而𝑒𝑝𝑜𝑠_𝑦範圍定義在-240~240之間、𝑑𝑒𝑝𝑜𝑠_𝑦範圍定義在-60~60之間。𝐾𝑓𝑝_𝑦(輸出)範圍定義在-0.003~0.003 之間。
  
X 軸速度之模糊自適應 P 控制器之模糊規則表：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure17.png)

- 由於UGV是以單方向移動(西方)，因此，在設計 X 軸速度之模糊自適應 P 控制器的模糊輸出時，必須讓UAV保持與UGV相同速度以維持跟隨，而當UAV超過UGV時，例如𝑒𝑝𝑜𝑠_𝑥為 PB 以及𝑑𝑒𝑝𝑜𝑠_𝑥為 PB 時，此時僅給予小幅地速度，即 PS，使無人機小幅度地往車子行走反方向進行修正。

Y 軸速度之模糊自適應 P 控制器之模糊規則表：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure18.png)

- 對於 y 軸之速度控制，只需穩定攝影機中心點保持在 ArUco marker 中心點上，若 Y 軸位置誤差𝑒𝑝𝑜𝑠_𝑦較大，且 Y 軸之位置誤差變化量𝑑𝑒𝑝𝑜𝑠_𝑦較大時，例如𝑒𝑝𝑜𝑠_𝑦為 NB 且𝑑𝑒𝑝𝑜𝑠_𝑦為 NB 時，會給予較大的反饋速度 NB，快速將無人機拉回至 ArUco marker 中心點位置。
  
## 5. 真實硬體飛行結果

起飛至 3 公尺高：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure16-a.png)

偏航校正控制：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure16-b.png)

PD 追蹤控制器：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure16-c.png)

模糊自適應 P 降落控制器：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure16-d.png)

無法檢測到 ArUco marker，啟動爬升機制：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure16-e.png)

爬升至重新檢測到 ArUco marker 時，快速拉回至上方：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure16-f.png)

繼續進行模糊自適應 P 降落控制器：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure16-g.png)

推力與姿態控制之降落控制，完成降落：

![image](https://github.com/Slaung/UAV-Autonomously-Tracking-Landing-on-UGV/blob/main/Figure/Figure16-h.png)

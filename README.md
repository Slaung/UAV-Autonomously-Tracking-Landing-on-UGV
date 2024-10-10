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
## 3. 視覺偵測
## 4. 控制器設計
## 5. 真實硬體飛行結果(成功、失敗、統計成功失敗次數)

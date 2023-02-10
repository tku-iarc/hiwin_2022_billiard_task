## Billiard Task in HIWIN Competition

### 上銀撞球指令
確認USB號碼：		ls /dev /tty*
連線USB：		sudo chmod 777 /dev/ttyUSB0 （最後一位數字根據上面確認之號碼）
進入工作資料夾：	cd /home/使用者名稱/hiwin_BilliardBall/src/libmodbus_ROS/src/My_test
執行校正程式：		python3 Correction_ball.py (執行後在教導器輸入新Z軸數值)
執行比賽程式：		python3 Hiwin_API_FianlVer.4.py


### 簡述

Hiwin_API_example.py：
	這份為學長給的手臂控制範例程式

Hiwin_API_test.py：
	整理了之前的程式
	
Hiwin_API_YoloTest.py：
	第一份測試程式，功能是將手臂移至拍照位置取得球桌資訊，找到母球後將手臂末端移至母球上方，
	可以移至母球處但不準

Hiwin_API_YoloTest2.py：
	第二份測試程式，功能與第一份大致相同，差在這份在拍照位置結束後會降低到中間高度在近拍一
	次，目的是希望靠近一點拍能取得更精準的座標
	
Hiwin_API_PointMove.py：
	這份目的是方便移至拍照位置，功能只有移至拍照為拍照
	
Houghcircles.py：
	這份是計算霍夫圓
	
Hiwin_API_HoughCircel.py：
	第三份測試程式，將原本用yolo判斷球位置改成用cv2的霍夫圓來計算球心位置
	
YOLO_Detect.py：
	這份為用python來套用yolo訓練出的模組到圖片上，cfg資料夾內放的是yolo訓練出的權重檔、
	.data、.names、.cfg

Hiwin_API_test1.py：
	這份為Hiwin_API_test.py後第二次整理的進度融合去年計算。整理用，無用處

Hiwin_API_HitBallTest.py：
	測試電磁炮及開始按鈕是否能正常運作用
	
Hiwin_API_NoArmTest.py：
	這份是因為疫情無法使用手臂，去除掉手臂控制單純丟圖片計算目標球及母球，最後有將球桌上狀況可視化

Hiwin_API_OpenPool.py：
	開球測試
	
Hiwin_API_table.py：
	可視化測試
	
Hiwin_API_TargetAngleTest.py：
	這份為去除掉執行步驟(沒有去年的mission_trigger)，直接從起始位置開始執行計算並打擊，並未有開始按鈕及開球

Hiwin_API_ProcessTest.py:
	比賽準備時間的測試程式，現已併入比賽主程式

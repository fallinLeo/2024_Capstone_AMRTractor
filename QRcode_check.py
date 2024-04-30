import cv2
import zbar

# 카메라 열기
cap = cv2.VideoCapture(6)

# 스캐너 생성
scanner = zbar.Scanner()

while True:
    # 프레임 캡처
    ret, frame = cap.read()

    # QR 코드 스캔
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    symbols = scanner.scan(gray)

    # 스캔된 QR 코드 출력
    for symbol in symbols:
        print(f"QR Code: {symbol.data.decode('utf-8')}")

    # 프레임 출력
    cv2.imshow('QR Code Scanner', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 카메라 및 창 종료
cap.release()
cv2.destroyAllWindows()

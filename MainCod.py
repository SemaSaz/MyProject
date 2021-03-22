import cv2
import numpy as np
from imutils.video import VideoStream
import imutils
from time import sleep
import RPi.GPIO as GPIO          

# название окна подстройки
WINDOWNAME = "Настройка тона"

# минимальный размер контуров пятна
BLOBSIZE = 1500

# константы насыщенности и яркости
S_MIN = 29
S_MAX = 255
V_MIN = 148
V_MAX = 255
RECTCOLOR = (0, 255, 0)
RTHICK = 2
def checkSize(w, h):
    if w * h > BLOBSIZE:
        return True
    else:
        return False
def empty(a):
    pass
frameSize = (320, 240)
vs = VideoStream(src=0, usePiCamera=True, resolution=frameSize, framerate=32).start()
sleep(2)

def ReadMessenger():
	 # получаем кадр изображения
        image = vs.read()

        # получаем максимальный и минимальный тон из значения ползунка
        h_min = cv2.getTrackbarPos("Hue", WINDOWNAME) - 10
        h_max = cv2.getTrackbarPos("Hue", WINDOWNAME) + 10

        # определяем границы цвета в HSV
        lower_range = np.array([h_min, S_MIN, V_MIN])
        upper_range = np.array([h_max, S_MAX, V_MAX])

        # конвертируем изображение в HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # создаём маску выбранного цвета
        thresh = cv2.inRange(hsv, lower_range, upper_range)

        # побитово складываем оригинальную картинку и маску
        bitwise = cv2.bitwise_and(image, image, mask=thresh)

        # показываем картинку маски цвета
        cv2.imshow("bitwise", bitwise)

        # удаляем цвет из маски
        gray = cv2.cvtColor(bitwise, cv2.COLOR_BGR2GRAY)

        # ищем контуры в картинке
        contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


MotorOnePinFirst = 24
MotorOnePinSecond = 23
MotorEnForFirstMotor = 25

MotorTwoPinFirst = 20
MotorTwoPinSecond = 21
MotorEnForSecondMotor = 22

MotorThirdPinFirst = 17
MotorThirdPinSecond = 18
MotorEnForThirdMotor = 19

MotorFourthPinFirst = 16
MotorFourthPinSecond = 15
MotorEnForFourthMotor = 14

temp1=1

#FirstMotorPinSettings:
GPIO.setmode(GPIO.BCM)
GPIO.setup(MotorOnePinFirst,GPIO.OUT)
GPIO.setup(MotorOnePinSecond,GPIO.OUT)
GPIO.setup(MotorEnForFirstMotor,GPIO.OUT)
GPIO.output(MotorOnePinFirst,GPIO.LOW)
GPIO.output(MotorOnePinSecond,GPIO.LOW)

#SecondMotorPinSettings:
GPIO.setup(MotorTwoPinFirst,GPIO.OUT)
GPIO.setup(MotorTwoPinSecond,GPIO.OUT)
GPIO.setup(MotorEnForSecondMotor,GPIO.OUT)
GPIO.output(MotorTwoPinFirst,GPIO.LOW)
GPIO.output(MotorTwoPinSecond,GPIO.LOW)

#ThirdMotorPinSettings:
GPIO.setup(MotorThirdPinFirst,GPIO.OUT)
GPIO.setup(MotorThirdPinSecond,GPIO.OUT)
GPIO.setup(MotorEnForThirdMotor,GPIO.OUT)
GPIO.output(MotorThirdPinFirst,GPIO.LOW)
GPIO.output(MotorThirdPinSecond,GPIO.LOW)

#FourthMotorPinSettings:
GPIO.setup(MotorFourthPinFirst,GPIO.OUT)
GPIO.setup(MotorFourthPinSecond,GPIO.OUT)
GPIO.setup(MotorEnForFourthMotor,GPIO.OUT)
GPIO.output(MotorFourthPinFirst,GPIO.LOW)
GPIO.output(MotorFourthPinSecond,GPIO.LOW)


#Other:
p1=GPIO.PWM(MotorEnForFirstMotor,1000)
p1.start(25)
p2=GPIO.PWM(MotorEnForSecondMotor,1000)
p2.start(25)
p3=GPIO.PWM(MotorEnForThirdMotor,1000)
p3.start(25)
p4=GPIO.PWM(MotorEnForFourthMotor,1000)
p4.start(25)



def FunctionGoForward():
		GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
		GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)



while True:
        # если контуры найдены...
        if len(contours) != 0:
		

  

# закрываем все окна
cv2.destroyAllWindows()

# останавливаем видео поток
vs.stop()
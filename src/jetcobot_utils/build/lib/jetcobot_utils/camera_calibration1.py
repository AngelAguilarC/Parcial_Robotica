import cv2
camera=cv2.VideoCapture(0)
 
cv2.namedWindow('imgage', cv2.WINDOW_FREERATIO)
i = 0
flag=camera.isOpened()
camera.set(3,3840)
camera.set(4,2160)
camera.set(6,cv2.VideoWriter.fourcc(*'MJPG'))
print("L:{}".format(camera.get(3)),"H:{}".format(camera.get(4)),
      "FPS:{}".format(camera.get(cv2.CAP_PROP_FPS)))
print(camera.get(cv2.CAP_PROP_FOCUS))
while flag:
    (grabbed, img) = camera.read()
    cv2.imshow('imgage', img)
    if cv2.waitKey(1) & 0xFF == ord('j'):  # 按j保存一张图片
        i += 1
        u = str(i)
        firename=str('./photo/'+u+'.jpg')
        cv2.imwrite(firename, img)
        print('写入：',firename)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
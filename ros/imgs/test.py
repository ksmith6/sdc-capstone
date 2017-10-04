import cv2

for i in range(137):
   name = 'image' + str(i+1) + '.png'
   print(name)
   img = cv2.imread(name)
   
   #cv2.imshow('image', img)
   #cv2.waitKey(0)
   #rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
   cv2.imwrite('l' + name, img)


   

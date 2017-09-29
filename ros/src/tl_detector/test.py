import cv2
from keras.models import load_model
from numpy import zeros, newaxis
import rospkg
import numpy as np
import time

model = load_model('model.h5')

image = cv2.imread('test0.png')
imrs = cv2.resize(image, (400, 400)) 
imrs = imrs.astype(float)
imrs = imrs / 255.0

imrs = imrs[newaxis,:,:,:]
#print(imrs)
start_time=time.time() 

preds = model.predict(imrs)



#print('Predicted:' ,preds)
predicted_class = np.argmax(preds, axis=1)

end_time=time.time() 


print('Time:' ,end_time - start_time)
print('Predicted Class:' ,predicted_class)
lid = predicted_class[0]
 

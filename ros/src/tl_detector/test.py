import cv2
from keras.models import load_model
from numpy import zeros, newaxis
import rospkg
import numpy as np

model = load_model('model.h5')

image = cv2.imread('test.png')
imrs = cv2.resize(image, (400, 400)) 
imrs = imrs.astype(float)
imrs = imrs / 255.0

imrs = imrs[newaxis,:,:,:]
#print(imrs)
preds = model.predict(imrs)
#print('Predicted:' ,preds)
predicted_class = np.argmax(preds, axis=1)

print('Predicted Class:' ,predicted_class)
lid = predicted_class[0]
 

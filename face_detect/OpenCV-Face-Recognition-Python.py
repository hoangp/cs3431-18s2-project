import os
import numpy as np
import cv2
import time
import random


# function to capture pictures from camera using OpenCV
def capture_camera_img(data_folder, frame_width=800, frame_height=600, test_data_folder = 'test-data'):
    if not os.path.exists(data_folder): os.mkdir(data_folder)
    if not os.path.exists(test_data_folder): os.mkdir(test_data_folder)
    while True:
        name = input('Input the name:')
        if name == 'exit': 
            return
        else:
            dir_path = data_folder + '/' + name
            if not os.path.exists(dir_path): os.mkdir(dir_path)
            print(f'Folder {dir_path} is created!')
            
        stop_train = False
        print(f'Stoping capture training pictures by pressing "q":')
        capture = cv2.VideoCapture(0)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        while capture.isOpened():
                
            ret, frame = capture.read()
            
            #face, rect = detect_face(frame)
            faces, rects = detect_faces(frame)
            if not rects is None:
                for i in range(len(faces)):
                    draw_rectangle(frame, rects[i])
            else:
                draw_text(frame, "No face", 100, 100)
                continue
            
            cv2.namedWindow("camera", 0) # 参数取0可以拖动缩放窗口，为1不可以
            cv2.imshow("camera", frame)

            k = cv2.waitKey(1000) # 每帧数据延时1ms，延时为0读取的是静态帧  
            if k == ord('q') and not stop_train:
                # stop capturing train pictures
                stop_train = True
                dir_path = test_data_folder
                print(f'Stoping capture training pictures by pressing "q":')
            elif k == ord('q') and stop_train:
                # stop capturing test pictures
                break
            else:
                pass
                
            # screenshoot
            img_name = "screenshoot" + time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime()) +str(random.randint(0,1000)) + ".jpg"
            cv2.imwrite(dir_path + '/' + img_name, frame)
            print(f'Picture {img_name} is saved in {dir_path + "/" + img_name}')

        capture.release()
        cv2.waitKey(0)
        cv2.destroyAllWindows()


# function to detect face using OpenCV
def detect_face(img):
    #convert the test image to gray image as opencv face detector expects gray images
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    #load OpenCV face detector, I am using LBP which is fast
    #there is also a more accurate but slow Haar classifier
    #face_cascade = cv2.CascadeClassifier('opencv-files/lbpcascade_frontalface.xml')
    #face_cascade = cv2.CascadeClassifier("opencv-files/haarcascade_frontalface_default.xml")
    face_cascade = cv2.CascadeClassifier("opencv-files/haarcascade_frontalface_alt.xml")
    

    #let's detect multiscale (some images may be closer to camera than others) images
    #result is a list of faces
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5);
    
    #if no faces are detected then return original img
    if (len(faces) == 0):
        return None, None
    
    #under the assumption that there will be only one face,
    #extract the face area
    (x, y, w, h) = faces[0]
    
    #return only the face part of the image
    return gray[y:y+w, x:x+h], faces[0]
    
def detect_faces(img):
    #convert the test image to gray image as opencv face detector expects gray images
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    #load OpenCV face detector, I am using LBP which is fast
    #there is also a more accurate but slow Haar classifier
    #face_cascade = cv2.CascadeClassifier('opencv-files/lbpcascade_frontalface.xml')
    #face_cascade = cv2.CascadeClassifier("opencv-files/haarcascade_frontalface_default.xml")
    face_cascade = cv2.CascadeClassifier("opencv-files/haarcascade_frontalface_alt.xml")
    

    #let's detect multiscale (some images may be closer to camera than others) images
    #result is a list of faces
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5);
    
    #if no faces are detected then return original img
    if (len(faces) == 0):
        return None, None
    
    local_faces, boundboxs = [], []
    for f in faces:
        (x, y, w, h) = f
        local_faces.append(gray[y:y+w, x:x+h])
        boundboxs.append(f)
    
    
    return local_faces, boundboxs


# function to prepare training data (face, label)
def prepare_train_data(data_folder):
    '''
    data set: http://www.cs.columbia.edu/CAVE/databases/pubfig/download/dev_urls.txt
    
    training-data|
                 |Tom|
                     |1.pgm
                     |n.png
                 |Jack|
                     |1.pgm
                     |n.png      
                     
    '''
    if not os.path.exists(data_folder): os.mkdir(data_folder)
    faces = []
    
    labels = []
    dir_names = os.listdir(data_folder)
    names = dir_names
    for i, dir_name in enumerate(dir_names):
        name = dir_name
        label = i
        dir_path = data_folder + '/' + dir_name
        for image_name in os.listdir(dir_path):
            if image_name.startswith("."): continue  # ignore system files like .DS_Store
                   
            image_path = dir_path + "/" + image_name  # e.g training-data/Tom/1.pgm

            image = cv2.imread(image_path)
            
            #display an image window to show the image 
            cv2.imshow("Training on image...", image)
            cv2.waitKey(100)
            
            #detect face
            face, rect = detect_face(image)
            
            if face is not None: #ignore faces that are not detected
                faces.append(face)
                labels.append(label)
        
    cv2.destroyAllWindows()
    cv2.waitKey(1)
    cv2.destroyAllWindows()
    
    return faces, labels, names
        

# function to train model
def train(faces, labels):
    #create our LBPH face recognizer 
    face_recognizer = cv2.face.LBPHFaceRecognizer_create()
    #face_recognizer = cv2.face.EigenFaceRecognizer_create()
    #face_recognizer = cv2.face.FisherFaceRecognizer_create()

    #train our face recognizer of our training faces
    face_recognizer.train(faces, np.array(labels))
    
    return face_recognizer


# functions to test pictures
def draw_rectangle(img, rect):
    (x, y, w, h) = rect
    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
    
def draw_text(img, text, x, y):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)
    
def predict(face_recognizer, test_img, TH = 45):
    img = test_img.copy()
    #detect face from the image
    face, rect = detect_face(img)
    if rect is None: 
        print('No face in this picture')
        return None, None, None
    
    #print(face, rect)
    #predict the image using our face recognizer 
    label, confidence = face_recognizer.predict(face)
    if confidence > TH: 
        name = 'unknown'
    else:
        #get name of respective label returned by face recognizer
        name = names[label]
    
    
    #draw a rectangle around face detected
    draw_rectangle(img, rect)
    #draw name of predicted person
    draw_text(img, name, rect[0], rect[1]-5)
    print(label, confidence, name)
    return name, confidence, img 

def predict_imgs(face_recognizer, test_data_folder):
    if not os.path.exists(test_data_folder): os.mkdir(test_data_folder)
    for img_name in os.listdir(test_data_folder):
        print("Predicting images...")

        #load test images
        test_img = cv2.imread(test_data_folder + '/' + img_name)
        print(test_data_folder + '/' + img_name)
        #perform a prediction
        
        name, conf, predicted_img = predict(face_recognizer, test_img)
        if predicted_img is None:
            continue
        print("Prediction complete")

        #display both images
        cv2.imshow(img_name, predicted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    
if __name__ == '__main__':
 
    print("Step 0: Capture pictures from camera...")
    capture_camera_img('training-data-1')
    print()

    print("Step 1: Preparing data...")
    faces, labels, names = prepare_train_data('training-data-1')
    print("Total faces: ", len(faces))
    print("Total labels: ", len(labels))
    print(names)
    print()

    print("Step 2: Train model...")
    face_recognizer = train(faces, labels)
    print()
    
    print("Step 3: test data...")
    predict_imgs(face_recognizer, 'test-data')
    
    print()
    
    
   





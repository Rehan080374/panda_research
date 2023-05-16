import os
from sklearn.model_selection import train_test_split
import tensorflow as tf
from tensorflow import keras
from keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D

# gpus = tf.config.experimental.list_physical_devices('GPU') 
# for gpu in gpus: 
#     tf.config.experimental.set_memory_growth(gpu, True)

# physical_devices = tf.config.list_physical_devices('GPU')
# tf.config.experimental.set_memory_growth(physical_devices[0], True)
# Set device placement to CPU
tf.config.set_visible_devices([], 'GPU')

# define the data directory and class names
data_dir = '/home/rehan/catkin_ws/src/panda_research/hand_classifier/data/train_data_bw'
class_names = ['push', 'pull', 'up', 'down', 'stop']
# class_names = ['1', '2', '3']

# create an empty list to hold the image paths and labels
data = []
image_shape=[480,640,3]
batch =30
# loop over each class and collect only first and last 50% of images
for class_name in class_names:
    # get the path to the class directory
    class_dir = os.path.join(data_dir, class_name)
    # get a list of all image file names in the class directory
    file_names = os.listdir(class_dir)
    # calculate the indices to select first and last 50% of images
    n_images = len(file_names)
    first_index = 0
    last_index = n_images - 1
    if n_images > 1:
        last_index = int(n_images/2)
    # loop over each image in the class directory
    for i, file_name in enumerate(file_names):
        # get the path to the image file
        file_path = os.path.join(class_dir, file_name)
        # add the image path and label to the data list if it's the first or last 50% of images
        if i == first_index or i == last_index:
            data.append((file_path, class_name))

# split the data into training and testing sets
train_data, test_data = train_test_split(data, test_size=0.2, random_state=42)

# create an ImageDataGenerator object for data augmentation and preprocessing
# train_datagen = ImageDataGenerator(samplewise_std_normalization=True,rescale=1./255, shear_range=0.2, zoom_range=0.2, vertical_flip=True,width_shift_range=0.2,height_shift_range=0.2,brightness_range=(0.5,1.5),rotation_range=(30))
train_datagen = ImageDataGenerator(rescale=1./255, shear_range=0.2, zoom_range=0.2,brightness_range=(0.5,1.5),rotation_range=(30))
# create a generator for training data
train_generator = train_datagen.flow_from_directory(data_dir, target_size=(image_shape[0], image_shape[1]), classes=class_names, 
                                                    batch_size=batch, shuffle=True)

# create an ImageDataGenerator object for preprocessing test data
test_datagen = ImageDataGenerator(rescale=1./255)

# create a generator for testing data
test_generator = test_datagen.flow_from_directory(data_dir, target_size=(image_shape[0], image_shape[1]), classes=class_names, 
                                                  batch_size=batch, shuffle=False)

# define the model architecture
model = Sequential()
model.add(Conv2D(32, kernel_size=(3, 3), activation='relu', input_shape=image_shape))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Conv2D(64, kernel_size=(3, 3), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Conv2D(128, kernel_size=(3, 3), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Flatten())
model.add(Dense(256, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(len(class_names), activation='softmax'))

# compile the model
model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])

# train the model on the training data
model.fit(train_generator, epochs=10)

# Save the model
model.save('/home/rehan/catkin_ws/src/panda_research/hand_classifier/model11/keras_model.h5')

# Save the labels to a text file
with open('/home/rehan/catkin_ws/src/panda_research/hand_classifier/model11/labels.txt', 'w') as f:
    for label in train_generator.class_indices:
        f.write(label + '\n')

# # Freeze the layers except for the last layer
# for layer in model.layers[:-1]:
#     layer.trainable = False

# # Check the trainable status of the individual layers
# for layer in model.layers:
#     print(layer, layer.trainable)

# # Re-compile the model
# model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])

# # Train only the last layer
# model.fit(train_generator, epochs=5)

# Evaluate the model on the testing data
test_loss, test_acc = model.evaluate(test_generator)
print('Test loss:', test_loss)
print('Test accuracy:', test_acc)

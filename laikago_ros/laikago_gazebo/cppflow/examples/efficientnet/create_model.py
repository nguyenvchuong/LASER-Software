import tensorflow as tf

model = tf.keras.applications.EfficientNetB0()

# Export the my_model to a SavedModel
model.save('my_model', save_format='tf')



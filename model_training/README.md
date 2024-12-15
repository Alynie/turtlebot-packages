The model is trained to classify six different hand gestures based on blacka and white images. The labels are: fist, open palm, okay sign, thumb and pinky, point left, point right. If a high confidence score is not reached in any of these labels, then the validation loop forces the classification to be no gesture.
# Setup Model Training
1. Download Gesture_Recognition_Model_Training.ipynb from this folder.
2. In your Google Drive, download the following directory to your Drive or set up a shortcut: [Training Directory](https://drive.google.com/drive/folders/1jc-2cZpSlfcVbh1l7J-NZC0B8syPossr?usp=sharing)
3. Likewise, set up a validation set from this directory: [Validation Directory](https://drive.google.com/drive/folders/1pq7OvHKynHlooWGfGIlt2tzS3n1N_pvn?usp=sharing)
4. Adjust the filepaths in the .ipynb file.
# Setup Model Validation
This can be done in a separate Jupyter Notebook (described below), or else done immediately after training which happens in the aforementioned training .ipynb file.
1. Download Model_Validation.ipynb from this folder.
2. In your Google Drive, download the following directory to your Drive or set up a shortcut: [Validation Directory](https://drive.google.com/drive/folders/1pq7OvHKynHlooWGfGIlt2tzS3n1N_pvn?usp=sharing) This is the same directory as described in Step 3 above.
3. Download the model weights for the model from this directory and add to your Google Drive(approx. 350MB): [Model Weights](https://drive.google.com/file/d/1kz315DDWlAJXhZu5a0LII6cAFeenP9qR/view?usp=sharing)
4. Adjust the filepaths in the .ipynb file.

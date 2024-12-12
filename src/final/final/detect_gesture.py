from detectron2.engine import DefaultTrainer
from detectron2.config import get_cfg
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
import cv2

class Gesture():
    def __init__(self):
        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        cfg.DATASETS.TRAIN = ("mdata1_train",)
        self.metadata_train = MetadataCatalog.get("mdata1_train").set(thing_classes=["fist", "palm", "ok", "tnf", "one_finger_left", "one_finger_right", "no_gesture"])
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")  

        cfg.MODEL.ROI_HEADS.NUM_CLASSES = 7
        cfg.MODEL.DEVICE='cpu'
        cfg.MODEL.WEIGHTS = "train/95percentBW.pth"  # path to the model we just trained
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.4   # set a custom testing threshold
        
        # cfg = get_cfg()
        # cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        # cfg.DATASETS.TRAIN = ("mdata1_train")
        # cfg.DATASETS.TEST = ()
        # cfg.DATALOADER.NUM_WORKERS = 2
        # cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")  # Let training initialize from model zoo
        # cfg.SOLVER.IMS_PER_BATCH = 2
        # cfg.SOLVER.BASE_LR = 0.001  # pick a good LR
        # cfg.SOLVER.MAX_ITER = 1750    # 300 iterations seems good enough for this toy dataset; you will need to train longer for a practical dataset
        # cfg.SOLVER.STEPS = []        # do not decay learning rate
        # cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = 64   # 256 is faster, and good enough for this toy dataset (default: 512)
        # cfg.MODEL.ROI_HEADS.NUM_CLASSES = 7
        # cfg.OUTPUT_DIR = weights_dir
        # cfg.SEED = 42
        
        self.predictor = DefaultPredictor(cfg)
        print("=================Finish Init========================")

    def detect_gesture(self,img,id):
        outputs = self.predictor(img)
        # v = Visualizer(img[:,:,::-1], metadata=self.metadata_train, scale=1.2)
        # out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
        # cv2.imwrite("outcome"+str(id)+".jpg", out.get_image()[:, :, ::-1])
        print('==========',id,'============')
        print(outputs["instances"].pred_classes)
        print(outputs["instances"].pred_boxes)

        class_index = None
        class_name = None

        for idx, coordinates in enumerate(outputs["instances"].pred_boxes):
            class_index = outputs["instances"].pred_classes[idx]
            class_name = self.metadata_train.thing_classes[class_index]
            print(class_name, class_index)
        return (class_name, class_index)

        # return outputs["instances"].pred_classes

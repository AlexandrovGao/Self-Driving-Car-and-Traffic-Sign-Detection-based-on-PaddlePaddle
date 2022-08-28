import cv2
import numpy as np
from paddle.inference import Config
from paddle.inference import create_predictor
import matplotlib.pyplot as plt

def resize(img, size1):
    # 图片预处理
    """ resize """
    tmp = float(size1) / min(img.shape[0], img.shape[1])
    retW = int(round(img.shape[1] * tmp))
    retH = int(round(img.shape[0] * tmp))
    retS = cv2.resize(img, (retW, retH))
    ret = cv2.resize(retS, (size1, size1))
    return ret

def preprocess(img, size1):
    M = [127.5, 127.5, 127.5]
    std = [127.5, 127.5, 127.5]
    # 重整化
    img = resize(img, size1)
    # 色彩复制
    img = img[:, :, ::-1].astype('float32').transpose((2, 0, 1))
    imgM = np.array(M).reshape((3, 1, 1))
    img_std = np.array(std).reshape((3, 1, 1))
    img -= imgM
    img /= img_std
    return img[np.newaxis, :]

def predict_config(model_file, params_file):
    # 加载 config
    config = Config()
    # 加载训练好的模型文件
    config.set_prog_file(model_file)
    config.set_params_file(params_file)
    # 默认使用CPU预测
    config.enable_use_gpu(500, 0)
    config.switch_ir_optim()
    config.enable_memory_optim()
    predictor = create_predictor(config)
    return predictor

def predict(image, predictor, target_size):
    img = preprocess(image, target_size)
    input_names = predictor.get_input_names()
    input_tensor = predictor.get_input_handle(input_names[0])
    input_tensor.reshape(img.shape)
    input_tensor.copy_from_cpu(img.copy())
    # 执行Predictor
    predictor.run()
    # 获取输出
    output_names = predictor.get_output_names()
    output_tensor = predictor.get_output_handle(output_names[0])
    output_data = output_tensor.copy_to_cpu()
    print("output_names", output_names)
    print("output_tensor", output_tensor)
    print("output_data", output_data)
    return output_data


if __name__ == '__main__':
    model_file = "model_dir/model.pdmodel"
    params_file = "model_dir/model.pdiparams"
    
    import random
    image = cv2.imread("C3.jpg")
    
    # 加载模型
    predictor = predict_config(model_file, params_file)
    res = predict(image, predictor, target_size=224)
    # 输出预测结果
    plt.figure()

    plt.title(res)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    plt.imshow(image.astype('uint8'))
    plt.axis('on')

    plt.show()
import cv2
import numpy as np
import logging
import torch
# mxVision 中的 Tensor 数据结构与推理接口
from mindx.sdk import Tensor
from mindx.sdk import base
# 模型前后处理相关函数
from det_utils import get_labels_from_txt, letterbox, scale_coords, nms, draw_bbox
# 串口通信函数
from control import ser_init, send_once, car_movement, cmd2msg


def main():
    logging.basicConfig(level=logging.DEBUG)

    # 变量初始化
    try:
        base.mx_init()
    except Exception as e:
        logging.error(f"MindX initialization failed: {e}")
        return
    DEVICE_ID = 0
    KNOWN_WIDTH_CM = 3.0
    FOCAL_LENGTH = 900
    model_path = '/root/motor_exp/yolo_detect/model/yolov5s_bs1_1108.om'
    # 初始化串口通信
    try:
        ser = ser_init()
        logging.info("Serial communication initialized successfully")
    except Exception as e:
        logging.error(f"Serial initialization failed: {e}")
        ser = None
    # 初始化摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        logging.error("Cannot open camera")
        return
    video_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    video_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    logging.info(f"Camera resolution: {video_width}x{video_height}")
    fps = 5
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    outfile = '/root/motor_exp/yolo_detect/video_result.mp4'
    writer = cv2.VideoWriter(outfile, fourcc, fps, (video_width, video_height))
    # 加载模型
    try:
        model = base.model(modelPath=model_path, deviceId=DEVICE_ID)
        logging.info("Model loaded successfully")
    except Exception as e:
        logging.error(f"Model loading failed: {e}")
        cap.release()
        if ser:
            ser.close()
        return


    try:
        frame_count = 0
        while(cap.isOpened()):
            ret, frame = cap.read()  # 此处 frame 为 bgr 格式图片
            if not ret:
                logging.error("Failed to read frame from camera")
                break


            # 数据前处理
            img, scale_ratio, pad_size = letterbox(frame, new_shape=[640, 640])  # 对图像进行缩放与填充，保持长宽比
            img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, HWC to CHW
            img = np.expand_dims(img, 0).astype(np.float16)  # 将形状转换为 channel first (1, 3, 640, 640)，即扩展第一维为 batchsize
            img = np.ascontiguousarray(img) / 255.0  # 转换为内存连续存储的数组
            img = Tensor(img) # 将numpy转为转为Tensor类


            # 模型推理
            try:
                output = model.infer([img])[0]  # 执行推理。输入数据类型：List[base.Tensor]， 返回模型推理输出的 List[base.Tensor]
            except Exception as e:
                logging.error(f"Inference failed: {e}")
                continue


            # 后处理
            output.to_host()  # 将 Tensor 数据转移到 Host 侧
            output = np.array(output)  # 将数据转为 numpy array 类型
            boxout = nms(torch.tensor(output), conf_thres=0.4, iou_thres=0.5)  # 利用非极大值抑制处理模型输出，conf_thres 为置信度阈值，iou_thres 为iou阈值

            if len(boxout) > 0 and boxout[0] is not None:
                pred_all = boxout[0].numpy()  # 转换为numpy数组
                scale_coords([640, 640], pred_all[:, :4], frame.shape, ratio_pad=(scale_ratio, pad_size))  # 将推理结果缩放到原始图片大小
                labels_dict = get_labels_from_txt('/root/motor_exp/yolo_detect/coco_names.txt')  # 得到类别信息，返回序号与类别对应的字典

                # 绘制检测框并计算距离和坐标
                for idx, box in enumerate(pred_all):
                    x1, y1, x2, y2, conf, class_id = box
                    class_id = int(class_id)

                    if conf > 0.4:  # 置信度阈值
                        # 计算边界框的宽度和高度（在原图中的像素值）
                        box_width = x2 - x1
                        box_height = y2 - y1

                        # 计算中心点坐标
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)

                        # 获取物体类别名称
                        class_name = labels_dict[class_id] if class_id in labels_dict else f"Class_{class_id}"

                        # 如果检测到粉色物体，计算距离和移动指令
                        if class_name.lower() == "pink":  # 适配可能的名称
                            # 计算距离（使用物体的宽度进行计算）
                            distance_cm = (KNOWN_WIDTH_CM * FOCAL_LENGTH) / box_width

                            # 计算小车移动指令
                            movement_command = car_movement(distance_cm, center_x, frame.shape[1])

                            # 发送指令
                            if ser:
                                try:
                                    msg = cmd2msg(movement_command)
                                    send_once(ser, msg)
                                    logging.info(f"Sent command: {msg} for movement: {movement_command}")
                                except Exception as e:
                                    logging.error(f"Serial communication failed: {e}")

                            # 在图像上绘制检测框
                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

                            # 在检测框上方显示类别、置信度、距离和移动指令
                            label = f'{class_name}: {conf:.2f}'
                            distance_label = f'Dist: {distance_cm:.1f}cm'
                            coord_label = f'({center_x}, {center_y})'
                            movement_label = f'Move: {movement_command}'

                            # 显示标签信息
                            cv2.putText(frame, label, (int(x1), int(y1) - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            cv2.putText(frame, distance_label, (int(x1), int(y1) - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                            cv2.putText(frame, coord_label, (int(x1), int(y1) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            cv2.putText(frame, movement_label, (int(x1), int(y1) - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                            # 打印到终端
                            logging.info(f'Detected {class_name} at center: ({center_x}, {center_y}), distance: {distance_cm:.1f} cm')
                        else:
                            # 对于非粉色物体，只显示框和标签，不计算距离
                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            label = f'{class_name}: {conf:.2f}'
                            cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 将推理结果写入视频
            writer.write(frame)
            # Optional: Display frame (uncomment if you want to see the live feed)
            # cv2.imshow('YOLO Detection with Distance', frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

    except KeyboardInterrupt:
        logging.info("Abort by user\n")
    finally:
        cap.release()
        writer.release()
        if ser:
            ser.close()
        # cv2.destroyAllWindows()  # Uncomment if displaying the frame

    logging.info('Save infer result success\n')


if __name__ == "__main__":
    main()
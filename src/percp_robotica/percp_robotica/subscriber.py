import rclpy # Python library for ROS 2
import cv2 # OpenCV library

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.optimize import linear_sum_assignment

class Bounding_box:
    def __init__(self):
        self.class_name = ""
        self.top = 0
        self.left = 0
        self.bottom = 0
        self.right = 0

def hungarian_algorithm(persons, epp):
    cost_matrix = [[0 for j in range(len(epp))] for i in range(len(persons))]
    rows=[]
    cols=[]
    #Crear la matriz de costo
    for i in range(len(persons)):
       for j in range(len(epp)):
          cost_matrix[i][j] = calcular_iou(persons[i], epp[j])
    #Aplicar el algoritmo húngaro
    if(len(cost_matrix) != 0):
      row_ind, col_ind = linear_sum_assignment(cost_matrix)
      #Extraer los valores del algoritmo hungaro aplicado  
      for i, j in zip(row_ind, col_ind):
        rows.append(i)
        cols.append(j)
    return rows, cols

def calcular_iou(person, epp):
    # Obtener coordenadas de los bounding boxes
    x1, y1, x2, y2 = person.left, person.top, person.right, person.bottom
    x3, y3, x4, y4 = epp.left, epp.top, epp.right, epp.bottom
    
    # Calcular coordenadas de intersección
    x_left = max(x1, x3)
    y_top = max(y1, y3)
    x_right = min(x2, x4)
    y_bottom = min(y2, y4)

    if x_right < x_left or y_bottom < y_top:
        return 0.0

    # Calcular área de intersección y de unión
    intersection_area = (x_right - x_left) * (y_bottom - y_top)
    bb1_area = (x2-x1) * (y2-y1)
    bb2_area = (x4-x3) * (y4-y3)
    union_area = bb1_area + bb2_area - intersection_area

    # Calcular IoU
    iou = intersection_area / union_area
    return iou

def draw_bounding_box(img, label, top, right, bottom, left):
  th=3 #Thickness bounding box
  if(label == "Person Segura"):
    r=0
    g=210
    b=94
  elif(label == "Person NO Segura"):
    r=255
    g=0
    b=0
  else:
    r=0
    g=0
    b=255
  font = cv2.FONT_HERSHEY_COMPLEX_SMALL 
  img2 = cv2.putText(img, label,(left, top-th), font, 0.8,(b,g,r),1)
  return cv2.rectangle(img2,(left,top),(right,bottom),(b,g,r), th)
   

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
       
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(Image, 'video_topic', self.listener_callback, 10)
    self.subscription # prevent unused variable warning
    
    
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    #Modelo
    self.modelYOLO = YOLO('PATH TO MODEL') #Dir modelo

    #Características de video
    #dimensions = [1280, 720]
    #frame_rate = int(30)
    #coder = cv2.VideoWriter_fourcc(*'DIVX')
    #name_video = "" #Dir output video
    #self.video =  cv2.VideoWriter(name_video, coder, frame_rate, dimensions)
    
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')

    #ret, frame = self.cap.read()
    current_frame = self.br.imgmsg_to_cv2(data)
    # Display image

    persons_detected = []

    epp_detected = []
       
    #Clasificación de personas
    results = self.modelYOLO(current_frame)
    for r in results:
      for box in r.boxes: #Se itera sobre los bounding box detectados
        b = box.xyxy[0].detach().cpu().numpy().copy()
        c = box.cls
        class_name = self.modelYOLO.names[int(c)]
        bounding_box_person = Bounding_box()
        bounding_box_epp = Bounding_box()
        if (class_name == 'Person'): #Person
          bounding_box_person.class_name = class_name
          bounding_box_person.left = int(b[0])
          bounding_box_person.top = int(b[1])
          bounding_box_person.right = int(b[2])
          bounding_box_person.bottom = int(b[3])
          persons_detected.append(bounding_box_person)
        elif(class_name == "Hardhat" or class_name =="Mask"): #Mask or Hardhat
          if(class_name == "Hardhat"):
            bounding_box_epp.class_name = "Casco"
          if(class_name == "Mask"):
            bounding_box_epp.class_name = "Mascara"
          bounding_box_epp.left = int(b[0])
          bounding_box_epp.top = int(b[1])
          bounding_box_epp.right = int(b[2])
          bounding_box_epp.bottom = int(b[3])
          epp_detected.append(bounding_box_epp)     
    
    #Se aplica el algoritmo hungaro para relacionar las personas con las epps
    persons_safe, epps_index = hungarian_algorithm(persons_detected, epp_detected)

    #Se crea un array para las personas no seguras
    persons_unsafe = []
    #Se identifica aquelals personas que no estén relacionados con ninguna epp
    for i in range(len(persons_detected)):
      safe = False
      #Se recorre
      for j in persons_safe:
        if i == j:
          safe = True
          break
      if not safe:
        persons_unsafe.append(i)
    
    image = current_frame
    #Dibuja el bounding box de cada epp detectada
    for i in range (len(epp_detected)):
      epp = epp_detected[i]
      image = draw_bounding_box(image, epp.class_name, epp.top, epp.right, epp.bottom, epp.left)
    
    #Dibuja el bounding box de cada persona no segura detectada
    for i in range(len(persons_unsafe)):
      person = persons_detected[persons_unsafe[i]]
      image = draw_bounding_box(image, "Person NO Segura", person.top, person.right, person.bottom, person.left)

    #Dibuja el bounding box de cada persona segura detectada
    for i in range(len(persons_safe)):
      person = persons_detected[persons_safe[i]]
      image = draw_bounding_box(image, "Person Segura", person.top, person.right, person.bottom, person.left)

    self.get_logger().info("Personas NO seguras: {}, Personas seguras {}".format(len(persons_unsafe), len(persons_safe)))

    #Mostrar imagen
    cv2.imshow("camera", image)
    
    
    #Guardar video
    #self.video.write(image)
    

     
    cv2.waitKey(1)
   
def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  image_subscriber = ImageSubscriber()
   
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
   
  # Destroy the node explicitly
  image_subscriber.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()





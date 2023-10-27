import cv2
import mediapipe as mp
import math

def main():
    # Inicialize o módulo MediaPipe Pose
    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)

    # Inicialize a câmera
    cap = cv2.VideoCapture(0)  # Use a câmera padrão (0) ou especifique o índice da câmera

    while cap.isOpened():
        #print('INSIDE')
        ret, frame = cap.read()

        if not ret:
            print('ERROR')
            continue

        # Converta o quadro para RGB para o MediaPipe
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Detecte os pontos do corpo no quadro
        results = pose.process(frame_rgb)

        # Desenhe os pontos do corpo no quadro
        if results.pose_landmarks:
            #print('LANDMARKS')
            mp_drawing = mp.solutions.drawing_utils
            mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            height = frame.shape[1]
            witdh = frame.shape[0]
            # Extraia as coordenadas dos pontos relevantes (ombro, pulso e cintura)
            #LEFT_SIDE_IMG
            point_12 = (round(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].x*witdh,2),
                        round(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].y*height,2))
            point_16 = (round(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].x*witdh,2),
                        round(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].y*height,2))
            point_24 = (round(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP].x*witdh,2),
                        round(results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP].y*height,2))
            
            point_11 = (round(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].x*witdh,2),
                        round(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].y*height,2))
            point_15 = (round(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].x*witdh,2),
                        round(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].y*height,2))
            point_23 = (round(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP].x*witdh,2),
                        round(results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP].y*height,2))

            #ANGLES
            theta_1 = calculate_3angle(point_12, point_16, point_24)
            theta_2 = calculate_3angle(point_11, point_15, point_23)

            angle_text1 = f"{theta_1:.1f}"
            angle_text2 = f"{theta_2:.1f}"
            cv2.putText(frame, angle_text1, (int(point_12[0]+10), int(point_12[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            cv2.putText(frame, angle_text2, (int(point_11[0]+10), int(point_11[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            
            if (theta_1 > 15 and theta_1 > theta_2) or point_15[0] < point_23[0]:
                #print("Pointing to My Left")
                #print('-------------------')
                cv2.putText(frame, "RIGHT", (15,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3)
            elif (theta_2 > 15 and theta_1 < theta_2) or point_16[0] > point_24[0]:
                #print("Pointing to My Right")
                #print('--------------------')
                cv2.putText(frame, "LEFT", (15,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 3)

        # Exiba o quadro com os pontos do corpo e o ângulo
        cv2.imshow('Body Points Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Libere os recursos
    cap.release()
    cv2.destroyAllWindows()
    pose.close()

def calculate_3angle(p1, p2, p3):
    vector_1 = (p2[0] - p1[0], p2[1] - p1[1])
    vector_2 = (p3[0] - p1[0], p3[1] - p1[1])

    dot_product = vector_1[0] * vector_2[0] + vector_1[1] * vector_2[1]
    magnitude_1 = math.sqrt(vector_1[0]**2 + vector_1[1]**2)
    magnitude_2 = math.sqrt(vector_2[0]**2 + vector_2[1]**2)

    theta = math.acos(dot_product / (magnitude_1 * magnitude_2))
    theta_degrees = math.degrees(theta)

    return theta_degrees

if __name__ == '__main__':
    main()
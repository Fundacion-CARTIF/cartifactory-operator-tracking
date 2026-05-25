# HandLandmarks3D.msg
# 21 landmarks 3D en el orden MediaPipe Hands (0..20).
# Frame: header.frame_id (normalmente el de depth aligned a color)
# Si valid[i] == false, landmarks[i] no es fiable (p.ej. depth inválido / fuera de imagen).

std_msgs/Header header

string id              # "hand_0", "hand_1", ...
uint8 handedness       # 0=UNKNOWN, 1=LEFT, 2=RIGHT (mismo convenio que en tu 2D si ya lo usas así)
float32 confidence     # score del clasificador de handedness (si lo tienes; si no, pon NaN o 0)

geometry_msgs/Point[21] landmarks
bool[21] valid
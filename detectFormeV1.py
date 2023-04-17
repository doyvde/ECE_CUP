import cv2

# Initialisation de la capture vidéo depuis la caméra du Raspberry Pi
cap = cv2.VideoCapture(0)

# Réglage de la résolution de la caméra du Raspberry Pi
cap.set(3, 640)
cap.set(4, 480)

# Définition du codec et des propriétés de la vidéo enregistrée
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('fleche.avi', fourcc, 20.0, (640, 480))

# Récupérer la largeur de la vidéo
frame_width = int(cap.get(3))

# Boucle de capture et traitement de la vidéo
while True:
    # Lire une image de la vidéo
    ret, frame = cap.read()
    
    if ret:
        # Convertir l'image en niveaux de gris
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Appliquer une détection de contours sur l'image en niveaux de gris
        edges = cv2.Canny(gray, 50, 150)
        
        # Trouver les contours de l'image
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Parcourir les contours trouvés
        for cnt in contours:
            # Approximer le contour par une forme polygonale
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            
            # Si la forme polygonale a quatre côtés
            if len(approx) == 4:
                # Calculer les coordonnées du rectangle encadrant la flèche
                x, y, w, h = cv2.boundingRect(cnt)
                
                # Déterminer la direction de la flèche en fonction de la position du rectangle encadrant la flèche
                if x < frame_width/2:
                    direction = "gauche"
                else:
                    direction = "droite"
                
                # Afficher la direction de la flèche dans la console
                print("La flèche pointe vers la", direction)
                
                # Dessiner le rectangle sur l'image originale
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                
                # Afficher la direction de la flèche sur l'image
                cv2.putText(frame, direction, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        # Écrire la frame encadrée dans la vidéo
        out.write(frame)
        
        # Afficher la frame encadrée
        cv2.imshow('Frame encadrée', frame)
    
    # Si la touche 'q' est pressée, sortir de la boucle
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Fermer les fenêtres et libérer les ressources
cap.release()
out.release()
cv2.destroyAllWindows()


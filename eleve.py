class Eleve():

	def __init__ (self, nom, prenom):
		self.nom = nom
		self.prenom = prenom
		self.note_maths = 0.0
		self.note_ece_cup = 0.0
		self.note_electronique = 0.0

	def calcul_moyenne(self):
    		return (self.note_maths + self.note_ece_cup + self.note_electronique)/3
    

while True:
    
	nom_eleve = input("nom : ")
	prenom_eleve = input("prénom : ")

	eleve = Eleve(nom = nom_eleve, prenom = prenom_eleve)
	print ("L'eleve s'apelle ", eleve.nom, eleve.prenom)

	note_1 = float(input("saisir la note aux QCMs "))
	eleve.note_ece_cup = note_1
	print ("Il a eu ", eleve.note_ece_cup, " en QCM")


	note_2 = float(input("saisir la note aux démos "))
	eleve.note_maths = note_2
	print ("Il a eu ", eleve.note_maths, " aux démos.")
    
	note_3 = float(input("saisir la note pour les rapports "))
	eleve.note_electronique = note_3
	print ("Il a eu ", eleve.note_electronique, " aux rapports")


	print ("La moyenne de", eleve.nom, eleve.prenom,"est : ", eleve.calcul_moyenne())

import subprocess

def Operate():
        while True:
            print("Elija una opción:")
            print("1. Autonomous")
            print("2. Manual Control")
            print("3. Exit")
            
            opcion = input("Operation: ")
            
            if opcion == '1':
                try:
                    subprocess.run(["python", "C:\\Users\\Miguel\\OneDrive\\Escritorio\\autonomous_drone-main\\scripts\\trycamera.py"])
                    print("launching trycamera.py...")
                except FileNotFoundError:
                    print("El archivo trycamera.py no se encuentra en la ubicación especificada.")
            elif opcion == '2':
                print("Manual Control")
                try:
                    subprocess.run(["python", "C:\\Users\\Miguel\\OneDrive\\Escritorio\\autonomous_drone-main\\scripts\\teleop.py"])
                    print("launching teleop.py...")
                except FileNotFoundError:
                    print("El archivo trycamera.py no se encuentra en la ubicación especificada.")

            elif opcion == '3':
                print("Exit program...")
                break
            else:
                print("Invalid")

if __name__ == "__main__":
    Operate()
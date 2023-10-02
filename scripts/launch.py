import subprocess

def Operate():
        while True:
            print("Elija una opción:")
            print("1. Autonomous")
            print("2. Manual Control")
            print("3. Salir")

            opcion = input("Ingrese su elección: ")

            if opcion == '1':
                print("Ejecutando trycamera.py...")
                subprocess.run(["python", "trycamera.py"])
            elif opcion == '2':
                print("Seleccionó: Manual Control")
            elif opcion == '3':
                print("Saliendo del programa...")
                break
            else:
                print("Opción no válida. Por favor, ingrese 1, 2 o 3.")

if __name__ == "__main__":
    Operate()
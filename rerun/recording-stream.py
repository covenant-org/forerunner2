## TO DO: Instalar python y tkinter(python3-tk con apt) en docker

import tkinter as tk

def on_click():
    lbl.config(text="¡Hola, tkinter!")

root = tk.Tk()
root.title("Demo")
lbl = tk.Label(root, text="Pulsa el botón")
lbl.pack(padx=10, pady=10)
btn = tk.Button(root, text="Saludar", command=on_click)
btn.pack(padx=10, pady=10)
root.mainloop()
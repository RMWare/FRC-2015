from tkinter import *
from PIL import ImageTk, Image
from networktables import NetworkTable

# To see messages from networktables, you must setup logging
import logging
logging.basicConfig(level=logging.DEBUG)

root = Tk()


def scale(img, width):
    width_percent = (width / float(img.size[0]))
    height = int((float(img.size[1]) * float(width_percent)))
    img = img.resize((width, height), Image.ANTIALIAS)
    return img

stack_size = 150

tote_on  = ImageTk.PhotoImage(scale(Image.open("tote_on.png" ), stack_size))
tote_off = ImageTk.PhotoImage(scale(Image.open("tote_off.png"), stack_size))
can_on   = ImageTk.PhotoImage(scale(Image.open("can_on.png"  ), stack_size))
can_off  = ImageTk.PhotoImage(scale(Image.open("can_off.png" ), stack_size))


def update():
    if elevator.getBoolean('has_bin', False):
        can.configure(image=can_on)
    else:
        can.configure(image=can_off)

    num_totes = int(elevator.getNumber('tote_count', 0))
    for tote in totes:
        tote.configure(image=tote_off)

    for index in range(num_totes):
        totes[4 - index].configure(image=tote_on)

    if elevator.getBoolean('has_game_piece', False):
        totes[-1].configure(image=tote_on)

    root.after(100, update)

if __name__ == "__main__":
    can = Label(root, image=can_off)
    can.grid(row=0)
    totes = []
    for i in range(6):
        totes.append(Label(root, image=tote_off))
        totes[i].grid(row=i + 1)

    NetworkTable.setIPAddress("roborio-865.local")
    NetworkTable.setClientMode()
    NetworkTable.initialize()
    elevator = NetworkTable.getTable("Elevator")
    update()
    root.mainloop()

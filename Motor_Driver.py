import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tkinter import *
import numpy as np
import time
from time import sleep
import os,sys
import math

#Pi specific libraries
import RPi.GPIO as GPIO
import rpi_hardware_pwm as hwpwm
from rpi_hardware_pwm import HardwarePWM

Font_tuple = ("Arial", 10)
Font_tuple_big = ("Arial", 16, "bold")

"__________ Initialize GPIO ____________"
GPIO.setmode(GPIO.BCM)
INPUT_PIN = 23
GPIO.setup(INPUT_PIN, GPIO.IN)

"__________Initialize PWM_______________"
pwm = HardwarePWM(pwm_channel=0, hz=50, chip=0)
pwm.start(5)

def Aquisition(Register_Size, Time_Interval, INPUT_PIN, ax2):
        global Raw_Optical
        t = time.time()
        Raw_Optical = np.empty([0, 2])
        cnt = 0
        t = time.time()
        while(cnt<Register_Size):
                Raw_Optical = np.vstack((Raw_Optical, [(time.time()-t), GPIO.input(INPUT_PIN)]))
                cnt = cnt+1
                sleep(Time_Interval)
    
def Fast_Fourier(Data):
        f = Data[:, 1]
        t = Data[:, 0]
        n = len(t)
        dt = (t[-1]-t[0])/n
        print(dt)
        fhat = np.fft.fft(f, n)
        PSD = fhat*np.conj(fhat) / n
        #freq = (1/(dt*n))*np.arange(n)
        freq = np.linspace(0, 1/dt, n)
        PSD[0] = 0
        indices = (PSD > 10) & (freq<100) #& (freq>1)
        PSDclean = indices*PSD
        fhat = indices*fhat
        new_f = np.fft.ifft(fhat)
        new_f = new_f.real
        
        RPM = freq[np.argmax(PSDclean)]
        
        return new_f, PSDclean, freq, RPM

def Real_Frequency(Data):
        f = Data[:, 1]
        t = Data[:, 0]
        n = len(t)
        dt = (t[-1]-t[0])/n
        
        Switching_Times = np.empty([0, 1])
        Intervals = np.empty([0, 1])

        for i in range(1, len(f)):
            if(f[i]-f[i-1]>0.5):
                Switching_Times = np.vstack((Switching_Times, t[i]))

        for i in range(1, len(Switching_Times)):
            Intervals = np.vstack((Intervals, Switching_Times[i]-Switching_Times[i-1]))

        if(len(Intervals)>0):
            RPM = 60/np.mean(Intervals)
        else:
            RPM = 0

        return RPM

def Apply_PWM(pulse_duration):
        frequency = 50
        duty_cycle = (pulse_duration/10)*frequency
        pwm.change_duty_cycle(duty_cycle)

def PID_Correction(Setpoint, Current_RPM):
    global Absolute_Error
    global Integral_Error
    global Derivative_Error

    Previous_Error = Absolute_Error
    Absolute_Error = Setpoint - Current_RPM

    Integral_Error = Integral_Error + Absolute_Error
    Derivative_Error = Absolute_Error - Previous_Error

    Kp = 0.0001
    Ki = 0.00005
    Kd = 0

    Correction = Kp*Absolute_Error + Ki*Integral_Error + Kd*Derivative_Error

    return Correction

class Application(Tk):
    def __init__(self):
        super().__init__()
        self.configure(background = 'purple')
        self.geometry('800x450')
        self.create_widgets()
        self.main()
   
    def create_widgets(self):
        Plot_frame = Frame(self, bg='grey', width=575, height=430, bd=5)
        Plot_frame.place(x=10,y=10)
        Plot_frame.pack_propagate(False)

        Control_frame = Frame(self, bg='white', width=195, height=430, bd=5)
        Control_frame.place(x=595,y=10)
        Control_frame.pack_propagate(False)
        Control_frame.grid_propagate(False)

        fig = plt.figure(figsize=(6.4, 5.4))


        ax1 = plt.axes((0.1, 0.45, 0.85, 0.5))
        ax2 = plt.axes((0.1, 0.1, 0.85, 0.25))
        
        canvas = FigureCanvasTkAgg(fig, master=Plot_frame)
        canvas.get_tk_widget().pack()

        Status_Indicator = Label(Control_frame, text="Idle", height = 2, width = 9, background='white', fg='green', anchor="w")
        Status_Indicator.place(x=90, y=380)
        Status_Indicator.configure(font=Font_tuple_big)
        
        #plt.axes(ax2)
        Run_Button = Button(Control_frame, height = 1, width = 6, text="Run", command=lambda: self.Run(canvas, ax1, ax2, b, Status_Indicator))
        Run_Button.place(x=5, y=5)

        Stop_Button = Button(Control_frame, height = 1, width = 6, text="Stop", command=lambda: self.auto_plot(ax1))
        Stop_Button.place(x=100, y=5)

        Calibrate_Button = Button(Control_frame, height = 1, width = 18, text="Calibrate", command=lambda: self.Single_Shot(canvas, ax1, ax2, b, Status_Indicator))
        Calibrate_Button.place(x=5, y=40)

        Output_Button = Button(Control_frame, height = 1, width = 18, text="Output Data", command=lambda: self.Write_Data())
        Output_Button.place(x=5, y=75)
        
        Labs = []
        Labs.append(Label(Control_frame, text="Index", height = 1, width = 6, background='white', anchor="w"))
        Labs[0].place(x=0, y=105)


        Labs.append(Label(Control_frame, text="RPM", height = 1, width = 6, background='white', anchor="w"))
        Labs[-1].place(x=40, y=105)

        Labs.append(Label(Control_frame, text="Duration", height = 1, width = 8, background='white', anchor="w"))
        Labs[-1].place(x=105, y=105)

        h = 12
        w = 2
        b = []

        for i in range(h):
            Labs.append(Label(Control_frame, text=i+1, height = 1, width = 2, background='white', anchor="w"))
            Labs[-1].place(x=0, y=(130+i*20))

        for i in range(h):
            for j in range(w):
                b.append(Entry(Control_frame, width = 7, text=""))
                b[-1].place(x=(40+j*70), y=(130+i*20))


        Labs.append(Label(Control_frame, text="Disc status:", height = 1, width = 10, background='white', anchor="w"))
        Labs[-1].place(x=0, y=400)

        for i in range(len(Labs)):
            Labs[i].configure(font=Font_tuple)


        global PID_TF
        PID_TF = IntVar()

        CB = Checkbutton(Control_frame, text='PID Control', bd = 0, background='white',  height =1, width = 10, anchor="w", variable=PID_TF)
        CB.place(x = -5, y = 380)
        CB.toggle()
        CB.configure(font=Font_tuple)


    def main(self):
        global Raw_Optical
        global RPM_Value
        global Raw_Optical_Store

        global Absolute_Error
        global Integral_Error
        global Derivative_Error

        global Signal

        Raw_Optical = np.empty([0, 2])
        RPM_Value = np.empty([0, 2])
        Raw_Optical_Store = np.empty([0, 0])

        Absolute_Error = 0
        Integral_Error = 0
        Derivative_Error = 0

        Signal = np.empty([0, 1])

    def Run(self,canvas, ax1, ax2, b, Status_Indicator):
        # Pull global variables
        global Raw_Optical
        global Raw_Optical_Store
        global RPM_Value    

        global Absolute_Error
        global Integral_Error
        global Derivative_Error
        
        global Signal

        # Reset RPM and optical data at begining of run
        RPM_Value = np.empty([0, 2])
        Raw_Optical_Store = np.empty([0, 0])
        Signal = np.empty([0, 1])
        
        # Get rpm and duration values from entry boxes
        RPMs = []
        Durations = []
        for i in range(len(b)):
            if(b[i].get()==""):
                break
            if(i%2==0):
                RPMs.append(b[i].get())
            else:
                Durations.append(b[i].get())
        
        RPMs = np.asarray(RPMs, dtype=float)
        Durations = np.asarray(Durations, dtype=float)

        PWMs = (RPMs+23927.2)/21876.6
        PWMs[PWMs<1.05]=1.05
        PWMs[PWMs>1.3]=1.3

        #Begin Run Sequence
        Sequence_Start = time.time()
        Status_Indicator.configure(text='Running')
        Status_Indicator.configure(fg='red')
        for i in range(len(RPMs)):
            t = time.time()
            Store_Data = 1
            Signal = np.vstack((Signal, PWMs[i]))
            Apply_PWM(PWMs[i])
            while((time.time()-t)<Durations[i]):
                Aquisition(2000, 0.0001, 23, ax2)
                #new_f, PSD, freq, Current_RPM = Fast_Fourier(Raw_Optical)
                Current_RPM = Real_Frequency(Raw_Optical)
                if((time.time()-t)>4):
                    Correction = PID_Correction(RPMs[i], Current_RPM)
                    Signal[Signal>1.3] = 1.3
                    Signal = np.vstack((Signal, PWMs[i] + Correction))
                    Apply_PWM(Signal[-1])

                RPM_Value = np.vstack((RPM_Value, [(time.time()-Sequence_Start), Current_RPM])) 
                
                if(((time.time()-t)>(Durations[i]/2))&Store_Data):
                    Store_Data=0
                    if(i==0):
                        Raw_Optical_Store = Raw_Optical
                    else:
                        Raw_Optical_Store = np.column_stack((Raw_Optical_Store, Raw_Optical))

                plt.axes(ax2)
                ax2.clear()
                #ax2.scatter(RPM_Value[:, 0], RPM_Value[:, 1])
                ax2.plot(Raw_Optical[:, 0], Raw_Optical[:, 1])
                #ax2.set_xlim([0, 0.5])
                canvas.draw()
            
                plt.axes(ax1)
                ax1.clear()
                #ax1.plot(freq, PSD)
                #ax1.set_xlim([0, 100])
                ax1.scatter(RPM_Value[:, 0], RPM_Value[:, 1])
                #ax1.scatter(RPM_Value[:, 0], Signal[1:])
                canvas.draw()            
            
                self.update()
                sleep(0.5)

        pwm.change_duty_cycle(5)
        Status_Indicator.configure(text='Idle')
        Status_Indicator.configure(fg='green')
        self.after(500) # display plot keeping button resposive

    def Single_Shot(self,canvas, ax1, ax2, b, Status_Indicator):
        # Pull global variables
        global Raw_Optical
        global Raw_Optical_Store
        global RPM_Value

        # Reset RPM and optical data at begining of run
        RPM_Value = np.empty([0, 2])
        Raw_Optical_Store = np.empty([0, 0])
        
        # Get rpm and duration values from entry boxes
        RPMs = []
        Durations = []
        for i in range(len(b)):
            if(b[i].get()==""):
                break
            if(i%2==0):
                RPMs.append(b[i].get())
            else:
                Durations.append(b[i].get())
        
        RPMs = np.asarray(RPMs, dtype=float)
        Durations = np.asarray(Durations, dtype=float)

        #Begin Run Sequence
        Sequence_Start = time.time()
        Status_Indicator.configure(text='Running')
        Status_Indicator.configure(fg='red')

        for i in range(len(RPMs)):
            t = time.time()
            Store_Data = 1
            Apply_PWM(RPMs[i])
            while((time.time()-t)<Durations[i]):
                Aquisition(2000, 0.0001, 23, ax2)
                #new_f, PSD, freq, Current_RPM = Fast_Fourier(Raw_Optical)
                Current_RPM = Real_Frequency(Raw_Optical)
                RPM_Value = np.vstack((RPM_Value, [(time.time()-Sequence_Start), Current_RPM])) 
                
                if(((time.time()-t)>(Durations[i]/2))&Store_Data):
                    Store_Data=0
                    if(i==0):
                        Raw_Optical_Store = Raw_Optical
                    else:
                        Raw_Optical_Store = np.column_stack((Raw_Optical_Store, Raw_Optical))

                plt.axes(ax2)
                ax2.clear()
                #ax2.scatter(RPM_Value[:, 0], RPM_Value[:, 1])
                ax2.plot(Raw_Optical[:, 0], Raw_Optical[:, 1])
                canvas.draw()
            
                plt.axes(ax1)
                ax1.clear()
                ax1.scatter(RPM_Value[:, 0], RPM_Value[:, 1])
                #ax1.scatter(RPM_Value[:, 0], Signal[1:])
                canvas.draw()            
            
                self.update()
                sleep(0.5)

        pwm.change_duty_cycle(5)
        Status_Indicator.configure(text='Idle')
        Status_Indicator.configure(fg='green')         
        self.after(500) # display plot keeping button resposive

    def Write_Data(self):
        global Raw_Optical_Store
        global Raw_Optical
        global RPM_Value

        global Signal
    
        np.savetxt("Raw_Optical.txt", Raw_Optical_Store, delimiter=",")
        np.savetxt("RPM_Value.txt", RPM_Value, delimiter=",")
        np.savetxt("Signal.txt", Signal)

Application().mainloop()

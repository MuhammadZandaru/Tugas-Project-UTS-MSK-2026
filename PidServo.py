#-----------------Muhammad Zandaru---------------------------------#
#NIM : 26/592461/PPA/07477
#Tugas Proyek UTS Kendali Servo Sederhana


import numpy as np
import matplotlib.pyplot as plt


dt = 0.01
waktu = np.arange(0, 5, dt)
setpoint = 90.0

# pemanggilan
jenis_plot = "P"


if jenis_plot == "P":
    daftar_pid = [
        (1.0, 0.0, 0.0),
        (1.5, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (2.5, 0.0, 0.0)
    ]

elif jenis_plot == "PI":
    daftar_pid = [
        (1.0, 0.1, 0.0),
        (1.5, 0.2, 0.0),
        (2.0, 0.3, 0.0),
        (2.5, 0.4, 0.0)
    ]

elif jenis_plot == "PID":
    daftar_pid = [
        (1.0, 0.1, 0.1),
        (1.3, 0.0, 0.3),
        (2.0, 0.1, 1.3),
        (2.5, 0.0, 0.5)
    ]

else:
    raise ValueError('jenis_plot harus "P", "PI", atau "PID"')

plt.figure(figsize=(12, 8))

for nomor, (Kp, Ki, Kd) in enumerate(daftar_pid, 1):

    posisi = 0.0
    kecepatan = 0.0
    integral = 0.0
    error_sebelumnya = setpoint
    data_posisi = []

    for t in waktu:
        error = setpoint - posisi
        integral += error * dt
        turunan = (error - error_sebelumnya) / dt

        # Perhitungan PID
        kendali = Kp * error + Ki * integral + Kd * turunan
        kendali = np.clip(kendali, -180, 180)

        # Model dinamika motor servo
        percepatan = 8.0 * kendali - 4.0 * kecepatan
        kecepatan += percepatan * dt
        posisi += kecepatan * dt
        posisi = np.clip(posisi, 0, 180)

        data_posisi.append(posisi)
        error_sebelumnya = error

    
    posisi = np.array(data_posisi)
    error = np.abs(setpoint - posisi)

    
    indeks = np.where(posisi >= 0.9 * setpoint)[0]
    rise = waktu[indeks[0]] if len(indeks) else np.nan

   
    overshoot = max(
        0, (np.max(posisi) - setpoint) / setpoint * 100
    )

    
    puncak = np.argmax(posisi)
    peak = waktu[puncak]

  
    indeks = np.where(error > 0.02 * setpoint)[0]

    if len(indeks) == 0:
        settling = waktu[0]
    elif indeks[-1] < len(waktu) - 1:
        settling = waktu[indeks[-1] + 1]
    else:
        settling = np.nan

    
    error_akhir = abs(setpoint - posisi[-1])

    # hasil grafik di terminal
    print(f"\n===== PERCOBAAN {nomor} - KONTROL {jenis_plot} =====")
    print(f"Kp                 : {Kp}")
    print(f"Ki                 : {Ki}")
    print(f"Kd                 : {Kd}")
    print(f"Rise time          : {rise:.3f} detik")
    print(f"Overshoot          : {overshoot:.2f} %")
    print(f"Peak time          : {peak:.3f} detik")
    print(f"Settling time      : {settling:.3f} detik")
    print(f"Steady-state error : {error_akhir:.3f} derajat")

    #Grafik 
    plt.subplot(2, 2, nomor)

    plt.plot(waktu, posisi, label="Posisi aktual")
    plt.axhline(
        90, color="red", linestyle="--", label="Setpoint 90°"
    )
    plt.xlabel("Waktu (detik)")
    plt.ylabel("Sudut (derajat)")
    plt.title(
        f"Kontrol {jenis_plot} - Percobaan {nomor}\n"
        f"Kp={Kp}, Ki={Ki}, Kd={Kd}"
    )

    # Interval sumbu waktu setiap 0,5 detik
    plt.xticks(np.arange(0, 5.1, 0.5))

    plt.grid(True)
    plt.legend()

plt.tight_layout()
plt.show()

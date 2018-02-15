#!/usr/bin/python3.6
"""Apogee Targeting System (ATS)
Written for GIT LIT, the Georgia Institute of Technology team in the
University Student Launch Intitiative (USLI) 2017-2018 competition.

Will Wills
Email: whwthree@gmail.com
Cellphone #: (678) 313-8206

Original Date:
Oct 13 2017
Last changed:
Feb 15 2018
"""
from multiprocessing import Process, Pipe, Queue
from motors import StepperMotor
from sense_hat import SenseHat
from ina219 import INA219
from ina219 import DeviceRangeError
import numpy as np
import serial
import time


def serial_altimeter(logger_q):
    """Process to open a serial connection with the
    StratologgerCF Altimeter and timestamp output before sending.
    
    Positional Argument:
        1) conn - Pipe to data formatting process

    Output to Pipe: Time-stamped serial data
    """
    # serial settings for RPi3 and StratologgerCF
    ser_port = '/dev/ttyS0'
    baud_rate = 9600
    data_bits = 8
    parity = serial.PARITY_NONE
    stop_bits = serial.STOPBITS_ONE
    # OnPad or OnLaunch setting for StratologgerCF telemetry
    #     0 - OnLaunch
    #     1 - OnPad
    tel_mode = 1

    ser = serial.Serial(ser_port, baudrate=baud_rate, bytesize=data_bits,
                        parity=parity, stopbits=stop_bits)

    try:
        logger_q.put(tel_mode)
        while True:
            alt = ser.readline()
            time = time.time()
            logger_q.put((time, alt))
    except AssertionError: # Queue closed
        pass
    finally:
        ser.close()


def data_formatter(logger_q, ats_q):
    """Formats and logs sensor data.
    
    Postional Arguments:
        1) alt_conn - Connection to the altimeter input process
        2) ats_conn - Connection to the controlling process
    """

    def append_data(log_file_path, out, start_time, alt_time, alt,
                    sense, inas):
        """Returns a numpy array with new data appended.
        Also writes data to a log file.
        
        Positional Arguments:
            1) log_file_path - the system path to the log file
            2) out - the numpy array to be appended to
            3) start_time - start time of the altimeter
            4) alt_time - current time corresponding to the altitude
            5) alt - current altitude
            6) sense - the SenseHat() object
            7) inas - list of the voltage sensor objects
        """

        sense_time = time.time() # seconds
        h = sense.get_humidity() # relative
        p = sense.get_pressure() # Millibars
        t_h = sense.get_temperature_from_humidity() # C
        t_p = sense.get_temperature_from_pressure() # C

        o = sense.get_orientation() # degrees
        o_p = o['pitch']
        o_r = o['roll']
        o_y = o['yaw']

        c = sense.get_compass_raw() # microTeslas
        c_x = c['x']
        c_y = c['y']
        c_z = c['z']

        g = sense.get_gyroscope_raw() # rad/s
        g_x = g['x']
        g_y = g['y']
        g_z = g['z']

        a = sense.get_accelerometer_raw() # Gs
        a_x = a['x']
        a_y = a['y']
        a_z = a['z']
        
        v_lipos = inas[0].supply_voltage() # Vs
        v_9v_1 = inas[1].supply_voltage()
        v_9v_2 = inas[2].supply_voltage()
        v_gps_batt = inas[3].supply_voltage()

        new_out = np.array([[alt_time - start_time, alt,
                             sense_time - start_time,
                             h, p, t_h, t_p,
                             o_p, o_r, o_y,
                             c_x, c_y, c_z,
                             g_x, g_y, g_z,
                             a_x, a_y, a_z,
                             v_lipos, v_9v_1, v_9v_2, v_gps_batt
                            ]])
        with open(log_file_path, 'ab') as log:
            np.savetxt(log, new_out, delimiter='\t')
        return np.r_[out, new_out]

    tel_mode = logger_q.get()
    # Sense HAT initialization
    sense = SenseHat()
    # Voltage sensors initialization
    shunt_ohm=0.1
    max_expected_current=0.4
    inas = [INA219(shunt_ohm, max_expected_current, address=hex_address)
           for hex_address in (0x40, 0x41, 0x44, 0x45)]
    for sensor in inas:
        sensor.configure(sensor.RANGE_16V)
    # initialize start time
    # and build a list of altimeter data (output from the queue)
    new_alt_data = [logger_q.get()]
    start_time = new_alt_data[0][0]
    # set up logging file
    date_str = time.strftime('%y%m%d_%Hh%Mm%Ss')
    log_file_path = '/home/pi/logs/ATS_Log_' + date_str
    header_list = ['# ATS Log\tDate: ', date_str]
    if tel_mode:
        # first altitude is relative to sea level
        # add it to logging file
        header_list.append('\tAltitude: ')
        ground_alt = new_alt_data[0][1].decode().strip()
        header_list.append(ground_alt)
    header_str = ''.join(header_list)
    with open(log_file_path, 'a') as log:
        # write second header line
        log.write(header_str + '\n')
        log.write('\t'.join(['# alt_time(s)', 'alt(ft)', 'sense_time(s)',
                            'h(r%)', 'p(millibars)', 't_h(C)', 't_p(C)',
                            'o_p(deg)', 'o_r(deg)', 'o_y(deg)',
                            'c_x(microTeslas)',
                            'c_y(microTeslas)',
                            'c_z(microTeslas)',
                            'g_x(rad/s)', 'g_y(rad/s)', 'g_z(rad/s)',
                            'a_x(G)', 'a_y(G)', 'a_z(G)', 'LiPOs(V)',
                            '9V_1(V)', '9V_2(V)', 'GPS_BATT(V)'
                            '\n']))

    if tel_mode:
        # get new data since the first point was sea level altitude
        new_alt_data = [logger_q.get() for i in range(logger_q.qsize())]
    else:
        # extend the current data set
        new_alt_data.extend([logger_q.get() for i in range(logger_q.qsize())])
    # save partial lines to a buffer
    buffer_list = []
    while True:
        out = np.empty((0, 23))
        # if the proces is behind dump every other data point
        if len(new_alt_data) > 9:
            new_alt_data = new_alt_data[::2]
        # parse new data
        for alt_data in new_alt_data:
            current_time = alt_data[0]
            alt = int(alt_data[1].decode().strip())
            out = append_data(log_file_path, out,
                              start_time, current_time, alt, sense, inas)
        # if there is no new data refresh and try again
        if out.size == 0:
            new_alt_data = [logger_q.get() for i in range(logger_q.qsize())]
            continue
        # output the new data
        try:
            ats_q.put(out)
        except AssertionError: # Queue closed
            logger_q.close()
            break
        # get new data
        new_alt_data = [logger_q.get() for i in range(logger_q.qsize())]


if __name__ == '__main__':
    # setup serial altimeter process
    logger_q = Queue()
    alt_p = Process(target=serial_altimeter, args=(logger_q,))
    alt_p.start()
    # setup logger process
    ats_q = Queue()
    logger_p = Process(target=data_formatter, args=(logger_q, ats_q))
    logger_p.start()
    # start main body
    motor = StepperMotor()
    motor.ready()
    # wait for altitude to be >= 500 AGL
    data = ats_q.get()
    alts = data[:,1]
    # find the first index where the altitude >= 500
    ndxs_500 = np.nonzero(alts >= 500)[0]
    current_ndx_500 = ndxs_500[0]
    ndx_500_num = 0
    # check all altitudes after the current_ndx_500 to be >= 500
    while np.average(alts[current_ndx_500:]) < 500:
        try:
            # check later data
            current_ndx_500 = ndxs_500[ndx_500_num + 1]
        except:
            # later data hasn't arrived yet
            data = ats_q.get()
            alts = data[:,1]
            ndxs_500 = np.nonzero(alts >= 500)[0]
            continue
        # later data exist -> increment
        ndx_500_num += 1
    # wait for negative or zero Gs
    vert_Gs = data[:,18]
    # find the first index where Gs <= 0
    ndxs_0G = np.nonzero(vert_Gs <= 0)[0]
    current_ndx_0G = ndxs_0G[0]
    ndx_0G_num = 0
    # check all Gs after the current_ndx_0G to be >= 0
    while np.average(vert_Gs[current_ndx_0G:] > 0):
        try:
            # check later data
            current_ndx_0G = ndxs_0G[ndx_0G_num + 1]
        except:
            # later data hasn't arrived yet
            data = ats_q.get()
            vert_Gs = data[:,18]
            ndxs_0G = np.nonzero(vert_Gs <= 0)[0]
            continue
        # later data exist -> increment
        ndx_0G_num += 1
    # collect drag coefficient data
    time.sleep(1)
    # iterate through deployment levels and collect drag data
    for i in range(5):
        m.deploy(10)
        time.sleep(1)
    m.retract(50)
    # wait for close to landing
    data = ats_q.get()
    alts = data[:,1]
    # find the first index where the altitude <= 500
    ndxs_500 = np.nonzero(alts <= 500)[0]
    current_ndx_500 = ndxs_500[0]
    ndx_500_num = 0
    # check all altitudes after the current_ndx_500 to be <= 500
    while np.average(alts[current_ndx_500:]) > 500:
        try:
            # check later data
            current_ndx_500 = ndxs_500[ndx_500_num + 1]
        except:
            # later data hasn't arrived yet
            data = ats_q.get()
            alts = data[:,1]
            ndxs_500 = np.nonzero(alts <= 500)[0]
            continue
        # later data exist -> increment
        ndx_500_num += 1
    ats_q.close()
    alt_p.join()
    logger_p.join()


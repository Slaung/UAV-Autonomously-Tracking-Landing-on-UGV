#! /usr/bin/env python2

import numpy as np

class Fuzzy_Controller_X_Gain:
    def __init__(self):
        self.gain = -0.001
        # test8 t=52 #
        ##### Fuzzy rules #####
        self.rules = {
            'NB_NB': 'NB', 'NB_NS': 'NB', 'NB_ZE': 'NB', 'NB_PS': 'NM', 'NB_PB': 'NM',
            'NS_NB': 'NB', 'NS_NS': 'NB', 'NS_ZE': 'NM', 'NS_PS': 'NM', 'NS_PB': 'NS',
            'ZE_NB': 'NB', 'ZE_NS': 'NB', 'ZE_ZE': 'NM', 'ZE_PS': 'NM', 'ZE_PB': 'NM',
            'PS_NB': 'NM', 'PS_NS': 'NS', 'PS_ZE': 'NS', 'PS_PS': 'ZE', 'PS_PB': 'PS',
            'PB_NB': 'NS', 'PB_NS': 'NS', 'PB_ZE': 'ZE', 'PB_PS': 'PS', 'PB_PB': 'PM',
        }

        self.rules_output = {
            'NB': '-0.002', 'NM': '-0.0015', 'NS': '-0.001', 'ZE': '0', 'PS': '0.001', 'PM': '0.002', 'PB': '0.003'
        }

        self.uA = np.empty(5)
        self.duA = np.empty(5)

    ##### Membership function #####
    def T_left(self, x, a, b):
        return np.maximum(0, np.minimum(float((b - x)) / (b - a), 1))

    def T_middle(self, x, a, b, c):
        return np.maximum(0, np.minimum(float((x - a)) / (b - a), float((c - x)) / (c - b)))

    def T_right(self, x, a, b):
        return np.maximum(0, np.minimum(float((x - a)) / (b - a), 1))


    ##### Fuzzy inference #####
    def fuzzy_inference(self, velocity, delta_velocity):
        ### e_pos ###
        self.uA[0] = self.T_left(velocity, -320, -160)
        self.uA[1] = self.T_middle(velocity, -320, -160, 0)
        self.uA[2] = self.T_middle(velocity, -160, 0, 160)
        self.uA[3] = self.T_middle(velocity, 0, 160, 320)
        self.uA[4] = self.T_right(velocity, 160, 320)

        ### de_pos ###
        self.duA[0] = self.T_left(delta_velocity, -80, -40)
        self.duA[1] = self.T_middle(delta_velocity, -80, -40, 0)
        self.duA[2] = self.T_middle(delta_velocity, -40, 0, 40)
        self.duA[3] = self.T_middle(delta_velocity, 0, 40, 80)
        self.duA[4] = self.T_right(delta_velocity, 40, 80)

        ### Fuzzy rule inference ###
        output = []
        output.append(self.rules['NB_NB'] if self.uA[0] > 0 and self.duA[0] > 0 else '')
        output.append(self.rules['NS_NB'] if self.uA[1] > 0 and self.duA[0] > 0 else '')
        output.append(self.rules['ZE_NB'] if self.uA[2] > 0 and self.duA[0] > 0 else '')
        output.append(self.rules['PS_NB'] if self.uA[3] > 0 and self.duA[0] > 0 else '')
        output.append(self.rules['PB_NB'] if self.uA[4] > 0 and self.duA[0] > 0 else '')

        output.append(self.rules['NB_NS'] if self.uA[0] > 0 and self.duA[1] > 0 else '')
        output.append(self.rules['NS_NS'] if self.uA[1] > 0 and self.duA[1] > 0 else '')
        output.append(self.rules['ZE_NS'] if self.uA[2] > 0 and self.duA[1] > 0 else '')
        output.append(self.rules['PS_NS'] if self.uA[3] > 0 and self.duA[1] > 0 else '')
        output.append(self.rules['PB_NS'] if self.uA[4] > 0 and self.duA[1] > 0 else '')

        output.append(self.rules['NB_ZE'] if self.uA[0] > 0 and self.duA[2] > 0 else '')
        output.append(self.rules['NS_ZE'] if self.uA[1] > 0 and self.duA[2] > 0 else '')
        output.append(self.rules['ZE_ZE'] if self.uA[2] > 0 and self.duA[2] > 0 else '')
        output.append(self.rules['PS_ZE'] if self.uA[3] > 0 and self.duA[2] > 0 else '')
        output.append(self.rules['PB_ZE'] if self.uA[4] > 0 and self.duA[2] > 0 else '')

        output.append(self.rules['NB_PS'] if self.uA[0] > 0 and self.duA[3] > 0 else '')
        output.append(self.rules['NS_PS'] if self.uA[1] > 0 and self.duA[3] > 0 else '')
        output.append(self.rules['ZE_PS'] if self.uA[2] > 0 and self.duA[3] > 0 else '')
        output.append(self.rules['PS_PS'] if self.uA[3] > 0 and self.duA[3] > 0 else '')
        output.append(self.rules['PB_PS'] if self.uA[4] > 0 and self.duA[3] > 0 else '')

        output.append(self.rules['NB_PB'] if self.uA[0] > 0 and self.duA[4] > 0 else '')
        output.append(self.rules['NS_PB'] if self.uA[1] > 0 and self.duA[4] > 0 else '')
        output.append(self.rules['ZE_PB'] if self.uA[2] > 0 and self.duA[4] > 0 else '')
        output.append(self.rules['PS_PB'] if self.uA[3] > 0 and self.duA[4] > 0 else '')
        output.append(self.rules['PB_PB'] if self.uA[4] > 0 and self.duA[4] > 0 else '')

        return output

    ##### Fuzzy matching #####
    def defuzzy(self, velocity_input, delta_velocity_input, output):
        ### Union ###
        w = []
        for i in range(5):
            for j in range(5):
                w.append(min(self.uA[j], self.duA[i]))

        ### Otuput ###
        y = []
        for i in range(25):
            key = output[i % len(output)]
            if key in self.rules_output:
                y.append(self.rules_output[key])
            else:
                y.append('0')

        ### defuzzy ###
        a = 0
        b = 0
        c = 0.0
        for i in range(25):
            a+=float(w[i])*float(y[i])
            b+=w[i]
        c = float(a) / b
        return c

    def inference(self, e_pos, de_pos):
        output = self.fuzzy_inference(e_pos, de_pos)
        self.gain = self.defuzzy(e_pos, de_pos, output)

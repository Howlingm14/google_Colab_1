import time
import numpy as np
import tkinter as tk
from PIL import ImageTk, Image
from scipy.stats import norm

np.random.seed(1)
PhotoImage = ImageTk.PhotoImage
UNIT = 50  # pixels
HEIGHT = 10  # grid height
WIDTH = 20  # grid width


anti_exploitation = np.ones((WIDTH, HEIGHT))

step_counter=0


class Env(tk.Tk):
    
    
    def __init__(self):
        super(Env, self).__init__()
        self.action_space = ['u', 'd', 'l', 'r']
        self.n_actions = len(self.action_space)
        self.title('Q Learning')
        self.geometry('{0}x{1}'.format(HEIGHT * UNIT, HEIGHT * UNIT))
        self.shapes = self.load_images()
        self.canvas = self._build_canvas()
        self.texts = []

        

    def _build_canvas(self):
        canvas = tk.Canvas(self, bg='white',
                           height=HEIGHT * UNIT,
                           width=WIDTH * UNIT)
        # create grids
        for c in range(0, WIDTH * UNIT, UNIT):  # 0~400 by 80
            x0, y0, x1, y1 = c, 0, c, HEIGHT * UNIT
            canvas.create_line(x0, y0, x1, y1)
        for r in range(0, HEIGHT * UNIT, UNIT):  # 0~400 by 80
            x0, y0, x1, y1 = 0, r, WIDTH * UNIT, r
            canvas.create_line(x0, y0, x1, y1)

        # add img to canvas
        self.rectangle = canvas.create_image(225, 25, image=self.shapes[0])
        self.triangle1 = canvas.create_image(175, 325, image=self.shapes[1])
        self.triangle2 = canvas.create_image(175, 375, image=self.shapes[1])
        self.triangle3 = canvas.create_image(375, 325, image=self.shapes[1])
        self.triangle4 = canvas.create_image(425, 225, image=self.shapes[1])
        self.triangle5 = canvas.create_image(425, 275, image=self.shapes[1])
        self.triangle6 = canvas.create_image(425, 325, image=self.shapes[1])
        self.triangle7 = canvas.create_image(475, 175, image=self.shapes[1])
        self.triangle8 = canvas.create_image(475, 225, image=self.shapes[1])
        self.triangle9 = canvas.create_image(475, 325, image=self.shapes[1])
        self.triangle10 = canvas.create_image(475, 375, image=self.shapes[1])
        self.triangle11= canvas.create_image(525, 225, image=self.shapes[1])
        self.triangle12 = canvas.create_image(675, 325, image=self.shapes[1])
        self.triangle13 = canvas.create_image(725, 275, image=self.shapes[1])
        self.triangle14 = canvas.create_image(725, 325, image=self.shapes[1])
        self.triangle15 = canvas.create_image(775, 225, image=self.shapes[1])
        self.triangle16 = canvas.create_image(775, 275, image=self.shapes[1])
        self.triangle17 = canvas.create_image(825, 275, image=self.shapes[1])
        self.triangle18 = canvas.create_image(825, 325, image=self.shapes[1])
        self.circle = canvas.create_image(225, 225, image=self.shapes[2])

        # pack all
        canvas.pack()

        return canvas

    def load_images(self):
        rectangle = PhotoImage(
            Image.open("C:/Users/User/Documents/Python Scripts/RL attempts/reinforcement-learning/1-grid-world/img/Drone-PNG.png").resize((30, 30)))
        triangle = PhotoImage(
            Image.open("C:/Users/User/Documents/Python Scripts/RL attempts/reinforcement-learning/1-grid-world/img/triangle.png").resize((30, 30)))
        circle = PhotoImage(
            Image.open("C:/Users/User/Documents/Python Scripts/RL attempts/reinforcement-learning/1-grid-world/img/circle.png").resize((30, 30)))

        return rectangle, triangle, circle

    def text_value(self, row, col, contents, action, font='Helvetica', size=9,
                   style='normal', anchor="nw"):

        if action == 0:
            origin_x, origin_y = 3, 16
        elif action == 1:
            origin_x, origin_y = 35, 16
        elif action == 2:
            origin_x, origin_y = 21, 3
        else:
            origin_x, origin_y = 21, 30

        x, y = origin_y + (UNIT * col), origin_x + (UNIT * row)
        font = (font, str(size), style)
        text = self.canvas.create_text(x, y, fill="black", text=contents,
                                       font=font, anchor=anchor)
        return self.texts.append(text)

    def print_value_all(self, q_table):
        for i in self.texts:
            self.canvas.delete(i)
        self.texts.clear()
        for i in range(WIDTH):
            for j in range(HEIGHT):
                for action in range(0, 4):
                    state = [i, j]
                    if str(state) in q_table.keys():
                        temp = q_table[str(state)][action]
                        self.text_value(j, i, round(temp, 2), action)

    def coords_to_state(self, coords):
        x = int((coords[0] - 25) / 50)
        y = int((coords[1] - 25) / 50)
        return [x, y]

    def state_to_coords(self, state):
        x = int(state[0] * 50 + 25)
        y = int(state[1] * 50 + 25)
        return [x, y]

    def reset(self):
        global anti_exploitation
        anti_exploitation = np.ones((HEIGHT, WIDTH))
        global reward_sum
        reward_sum=0
        global reward
        reward=0
        self.update()
        time.sleep(0.5)
        x, y = self.canvas.coords(self.rectangle)
        self.canvas.move(self.rectangle, UNIT / 2 - x+500, UNIT / 2 - y)
        self.render()
        # return observation
        return self.coords_to_state(self.canvas.coords(self.rectangle))


    def step(self, action):
        global reward
        global reward_sum
        global anti_exploitation
        state = self.canvas.coords(self.rectangle)
        base_action = np.array([0, 0])
        self.render()

        if action == 0:  # up
            if state[1] > UNIT:
                base_action[1] -= UNIT
        elif action == 1:  # down
            if state[1] < (HEIGHT - 1) * UNIT:
                base_action[1] += UNIT
        elif action == 2:  # left
            if state[0] > UNIT:
                base_action[0] -= UNIT
        elif action == 3:  # right
            if state[0] < (WIDTH - 1) * UNIT:
                base_action[0] += UNIT

        # move agent
        self.canvas.move(self.rectangle, base_action[0], base_action[1])
        # move rectangle to top level of canvas
        self.canvas.tag_raise(self.rectangle)
        next_state = self.canvas.coords(self.rectangle)

        # reward function
        
        if next_state in [self.canvas.coords(self.triangle1),
                            self.canvas.coords(self.triangle2),
                            self.canvas.coords(self.triangle3),
                            self.canvas.coords(self.triangle4),
                            self.canvas.coords(self.triangle5),
                            self.canvas.coords(self.triangle6),
                            self.canvas.coords(self.triangle7),
                            self.canvas.coords(self.triangle8),
                            self.canvas.coords(self.triangle9),
                            self.canvas.coords(self.triangle10),
                            self.canvas.coords(self.triangle11),
                            self.canvas.coords(self.triangle12),
                            self.canvas.coords(self.triangle13),
                            self.canvas.coords(self.triangle14),
                            self.canvas.coords(self.triangle15),
                            self.canvas.coords(self.triangle16),
                            self.canvas.coords(self.triangle17),
                            self.canvas.coords(self.triangle18)]:
            reward = -100
            done = True
        else:
            x=HEIGHT
            y=WIDTH
            sigmas_covered=3
            
            coords = (self.canvas.coords(self.rectangle))
            i = int((coords[0] - 25) / 50)
            j = int((coords[1] - 25) / 50)
            #print(i, j)
            
            u_x = -sigmas_covered+(2*(sigmas_covered)/x)*(j+1)
            l_x = -sigmas_covered+(2*(sigmas_covered)/x)*(j)
            cdf_upper_limit = norm(loc = 0 , scale = 1).cdf(u_x)
            cdf_lower_limit = norm(loc = 0 , scale = 1).cdf(l_x)
            p = cdf_upper_limit - cdf_lower_limit
            prob_x = p
            
            u_y = -sigmas_covered+(2*(sigmas_covered)/y)*(i+1)
            l_y = -sigmas_covered+(2*(sigmas_covered)/y)*(i)
            cdf_upper_limit = norm(loc = 0 , scale = 1).cdf(u_y)
            cdf_lower_limit = norm(loc = 0 , scale = 1).cdf(l_y)
            p = cdf_upper_limit - cdf_lower_limit
            prob_y = p
            
            p_box = prob_x*prob_y
            if i==0:
                p_box=0
            elif j==0:
                p_box=0
            else:
                p_box=p_box
                
            #print(p_box*100*anti_exploitation[j, i])
            reward_sum = reward_sum + p_box*100*anti_exploitation[j][i]
            
            reward = p_box*100*anti_exploitation[j][i]
            if anti_exploitation[j][i]==0:
                reward = -10
            anti_exploitation[j][i]=0
            
            
            done = False

        next_state = self.coords_to_state(next_state)
        return next_state, reward, done, reward_sum

    def render(self):
        time.sleep(0.03)
        self.update()

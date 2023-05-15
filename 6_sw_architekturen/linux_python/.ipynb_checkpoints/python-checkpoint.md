---
jupytext:
  formats: md:myst
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.14.5
kernelspec:
  display_name: Python 3 (ipykernel)
  language: python
  name: python3
---

# Python

Python ist zu einer sehr wichtigen High-Level-Programmiersprache geworden, die oft für Glue-Code verwendet wird. -- in der Robotik, aber auch bei der Verarbeitung von Daten in Daten-Pipelines.

Glue-Code: Zusammenfügen von Programmen und großen Softwarekomponenten mit unterschiedlichsten Schnittstellen

Python kann als einfache Skriptsprache verwendet werden, aber auch um komplexere Objektorientierte Programme zu schreiben.

**Eine Word Cloud aus den drei Robotergesetzen von Isaac Asimov:**

Die Datei `robotergesetzte.txt` enthält als Text die [drei Asimov'schen Gesetzte](https://de.wikipedia.org/wiki/Robotergesetze#Allgemeines).

% https://www.python-graph-gallery.com/wordcloud/
% Angepasst

```{code-cell} ipython3
# Libraries
from wordcloud import WordCloud
import matplotlib.pyplot as plt

# Read the content of the text file
text = open('robotergesetze.txt').read()

# Create the wordcloud object
wordcloud = WordCloud(width=480, height=480, margin=0).generate(text)

# Display the generated image:
plt.imshow(wordcloud, interpolation='bilinear')
plt.axis("off")
plt.margins(x=0, y=0)
plt.show()
```

**Eine Interaktive Karte der TH Aschaffenburg:** 

Die interaktive Funktion der Karte setzt eine Internetverbindung voraus.

```{code-cell} ipython3
# Import the folium library
import folium

thab_location = [49.971739, 9.161144];

# Build the default map for a specific location
map = folium.Map(
  location=thab_location,
  zoom_start=18
  )
folium.Marker(
  location=thab_location,
  popup='beste Hochschule weit und breit',
  ).add_to(map)
map
```

**Roboter**

```{code-cell} ipython3
import roboticstoolbox as rtb
robot = rtb.models.Panda()
print(robot)
```

```{code-cell} ipython3
Te = robot.fkine(robot.qr)  # forward kinematics
print(Te)
```

```{code-cell} ipython3
from spatialmath import SE3

Tep = SE3.Trans(0.6, -0.3, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
sol = robot.ik_lm_chan(Tep)         # solve IK
print(sol)
```

```{code-cell} ipython3
q_pickup = sol[0]
print(robot.fkine(q_pickup))    # FK shows that desired end-effector pose was achieved
```

```{code-cell} ipython3
sol
```

% https://github.com/petercorke/robotics-toolbox-python/wiki/Backends

```{code-cell} ipython3
#qt = rtb.jtraj(robot.qr, q_pickup, 50)
#robot.plot(qt.q, backend='pyplot')
#robot.plot(qt.q)
#pyplot.step()

%matplotlib notebook

from roboticstoolbox.backends.PyPlot import PyPlot
env = PyPlot()
# where BACKEND is PyPlot, VPython or Swift

env.launch()  # start the backend
env.add(robot)  # add a robot to the backend

# https://github.com/petercorke/robotics-toolbox-python/wiki/Backends#backend-api
for i in range(0, 10):
    robot.q = sol[0]
    env.step(1)
    env.getfrage()
```

```{code-cell} ipython3
%matplotlib widget

import roboticstoolbox as rp
import numpy as np

panda = rp.models.DH.Panda()
# initial pose
q= np.array([0.00138894 ,5.98736e-05,-0.30259058,   -1.6, -6.64181e-05,    1.56995,-5.1812e-05])
panda.q = q
# joint torque limits
t_max = np.array([87, 87, 87, 87, 20, 20, 20]) 
t_min = -t_max

# robot matrices
Jac = panda.jacob0(q)[:3,:]
# gravity torque
gravity = panda.gravload(q).reshape((-1,1))

# visualise panda
fig = panda.plot(q)
ax = fig.ax

ax.set_xlim([-1, 1.5])
ax.set_ylim([-1, 1.5])
ax.set_zlim([0, 1.5])
plt.legend()
plt.show()
fig.hold()
```

```{code-cell} ipython3
# creating 3d plot using matplotlib
# in python
 
# for creating a responsive plot
%matplotlib widget
 
# importing required libraries
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
 
# creating random dataset
xs = [14, 24, 43, 47, 54, 66, 74, 89, 12,
      44, 1, 2, 3, 4, 5, 9, 8, 7, 6, 5]
 
ys = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 6, 3,
      5, 2, 4, 1, 8, 7, 0, 5]
 
zs = [9, 6, 3, 5, 2, 4, 1, 8, 7, 0, 1, 2,
      3, 4, 5, 6, 7, 8, 9, 0]
 
# creating figure
fig = plt.figure()
ax = Axes3D(fig)
 
# creating the plot
plot_geeks = ax.scatter(xs, ys, zs, color='green')
 
# setting title and labels
ax.set_title("3D plot")
ax.set_xlabel('x-axis')
ax.set_ylabel('y-axis')
ax.set_zlabel('z-axis')
 
# displaying the plot
plt.show()
```

```{code-cell} ipython3
%matplotlib widget

from spatialmath import *

T = SE3(0.5, 0.0, 0.0) * SE3.RPY([0.1, 0.2, 0.3], order='xyz') * SE3.Rx(-90, unit='deg')

print(T)

T.eul()

T.R

T.t

T.plot(color='red', label='2')
```

%```{code-cell} ipython3
%import roboticstoolbox as rtb
%
%panda = rtb.models.URDF.Panda()
%rint(panda)
%panda.plot(panda.qz, backend="swift")
%```

%```{code-cell} ipython3
%import roboticstoolbox as rtb
%import vpython
%
%p560 = rtb.models.DH.Puma560()
%qt = rtb.tools.trajectory.jtraj(p560.qz, p560.qr, 50)
%#env = p560.plot(qt.q, backend='vpython')
%p560.plot(qt.q)
%```

**Interaktive Choropleth-Karte mit Zeitachse**

% https://nbviewer.org/github/plotly/plotly_express/blob/gh-pages/walkthrough.ipynb

```{code-cell} ipython3
import plotly.express as px 
gapminder = px.data.gapminder()
gapminder2007 = gapminder.query("year == 2007")

px.scatter(gapminder2007, x="gdpPercap", y="lifeExp")

px.choropleth(
  gapminder, 
  locations="iso_alpha", 
  color="lifeExp", 
  hover_name="country", 
  animation_frame="year",
  color_continuous_scale=px.colors.sequential.Plasma, 
  projection="natural earth"
  )
```

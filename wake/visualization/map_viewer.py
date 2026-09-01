def show_points(points):
    try:import matplotlib.pyplot as plt
    except ImportError as exc:raise RuntimeError("install wake-mapper[science] for visualization") from exc
    axes=plt.figure().add_subplot(projection="3d");points=list(points)
    if points:axes.scatter(*zip(*points));plt.show()

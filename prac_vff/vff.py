import numpy as np

def calculate_vff_force(obstacles):
    """
    Calculate the VFF force for a given list of obstacles.
    
    :param obstacles: List of obstacles where each obstacle is represented as [x, y, w, h]
    :return: The resulting VFF force as a numpy array
    """
    ship_position = np.array([0, 0])  # 선박의 현재 위치
    ship_direction = np.array([1, 0])  # 선박의 현재 방향

    # 장애물에서의 반발력을 계산
    repulsive_forces = []
    for obs in obstacles:
        center = np.array([obs[0], obs[1]])  # 장애물 중심
        distance = np.linalg.norm(center - ship_position)  # 선박과 장애물 사이 거리(스칼라)
        direction = (center - ship_position) / distance  # 선박에서 장애물로의 단위 벡터
        force_magnitude = 1 / distance**2  # 반발력의 크기 (거리의 제곱에 반비례)
        repulsive_forces.append(-force_magnitude * direction)  # 반발력 (반대 방향)

    # 모든 장애물에 대한 반발력의 합
    total_repulsive_force = np.sum(repulsive_forces, axis=0)
    print("output force : ", total_repulsive_force)
    return total_repulsive_force

# 예시 장애물 좌표
obstacles_example = [[5, 5, 1, 1], [10, -3, 2, 2]]

# VFF 합력 계산
vff_force = calculate_vff_force(obstacles_example)
vff_force
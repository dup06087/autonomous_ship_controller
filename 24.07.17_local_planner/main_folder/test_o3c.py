import open3d as o3d
import open3d.core as o3c

# GPU 사용 가능 여부 확인 함수
def is_gpu_available():
    try:
        # CUDA:0 디바이스가 사용 가능한지 테스트
        device = o3c.Device("CUDA:0")
        # Tensor를 GPU에 할당해보기
        tensor = o3c.Tensor([0.0, 1.0, 2.0], dtype=o3c.float32, device=device)
        print("Tensor on GPU:", tensor)
        return True
    except Exception as e:
        print("No GPU available:", e)
        return False

# GPU 가속 사용 가능 여부 확인
if is_gpu_available():
    print("GPU acceleration is available.")
else:
    print("GPU acceleration is not available.")

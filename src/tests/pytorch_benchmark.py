import torch
import torchvision.models as models
import time

if __name__ == "__main__":
    def current_ms(): return int(round(time.time() * 1000))
    model = models.resnet50(pretrained=True).cuda()
    model = model.eval()

    # Inference benchmarking
    # 500 iterations warm up, 1000 iterations taking average
    n_iters = 2000
    warmup_iters = 1000
    accum_ms = 0
    curr_iter = 0
    t_0 = current_ms()

    for i in range(n_iters):
        mock_input = torch.randn(1, 3, 224, 224)
        t_0 = current_ms()
        mock_input = mock_input.cuda()
        _ = model(mock_input)
        curr_iter += 1

        if (curr_iter > warmup_iters):
            accum_ms += current_ms() - t_0

    avg_ms = accum_ms / 1000.0
    print("Benchmarking 1000 iterations, Average: {} ms, {} Hz".format(
        avg_ms, 1000 / avg_ms))

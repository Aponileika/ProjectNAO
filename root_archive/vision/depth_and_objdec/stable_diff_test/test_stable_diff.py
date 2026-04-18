import torch


device = "mps" if torch.backends.mps.is_available() else "cpu"
print(f"device is {device}")

pipe = StableDiffusionPipeline.from_pretrained(
    "runwayml/stable-diffusion-v1-5",
    torch_dtype=torch.float16
)

pipe = pipe.to(device)

prompt = "a photorealistic futuristic city, sunset, ultra detailed"

with torch.autocast(device):
    image = pipe(prompt).images[0]

image.save("output.png")


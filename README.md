1. Install docker
```
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```
```
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
2. Add Docker to superuser group
```
sudo usermod -aG docker $USER
newgrp docker
```
3. Clone this repo
```
git clone --recurse-submodules git@github.com:5uvin/test_automation.git
```

4. Build Docker image
```
cd test_automation
docker build -t test_automation .
```
5. Create Container from image
```
docker run -it --name test_automation_container --network=host test_automation
```
6. Open bash shell in container
```
docker exec -it test_automation_container /bin/bash
```

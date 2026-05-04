# Basic Usage Of Nvim

## How to start

```bash
# After you build this image or pull it
# To generate the target container
docker-compose up -d

# And then execute it
docker-compose exec -it rmcs-develop zsh

# Enjoy your nvim
# This config is same with your system
nvim
```

## Trouble shooting

- sudo: unable to resolve host developer: Name or service not known

    ```bash
    echo "127.0.0.1 localhost developer" | sudo tee /etc/hosts
    ```

    ```

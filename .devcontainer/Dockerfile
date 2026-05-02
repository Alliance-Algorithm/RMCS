FROM qzhhhi/rmcs-develop:latest

SHELL ["/bin/bash", "-lc"]

USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
    nodejs npm && \
    npm install -g @openai/codex && \
    apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

USER ubuntu
WORKDIR /home/ubuntu

RUN curl -fsSL https://opencode.ai/install | bash && \
    if ! grep -q '/.opencode/bin' ~/.bashrc; then printf '\n# opencode\nexport PATH="$HOME/.opencode/bin:$PATH"\n' >> ~/.bashrc; fi && \
    if ! grep -q '/.opencode/bin' ~/.profile; then printf '\n# opencode\nexport PATH="$HOME/.opencode/bin:$PATH"\n' >> ~/.profile; fi && \
    if ! grep -q '/.opencode/bin' ~/.zshrc; then printf '\n# opencode\nexport PATH="$HOME/.opencode/bin:$PATH"\n' >> ~/.zshrc; fi && \
    mkdir -p ~/.opencode/skills

ENV PATH="/home/ubuntu/.opencode/bin:${PATH}"

# TO DO
Start training in a playground environment where we just reward distance traveled
Then move on to straight line following

Train a separate policy to predict next state given an action
Can train this at the same time as a policy that predicts action for a requested state

Then train a line follower with Q learning, asigning value based on following the closest segment

Along the process, we'll need to do multiple refactors to abstract away track/agent queries.

One of the important separations is the agent behavior. It'd be nice to be able to swap in more complex
agent dynamics (different steering, grip, gas/brake, etc).

Refs:
- Double Q Learning https://proceedings.neurips.cc/paper_files/paper/2010/file/091d584fced301b442654dd8c23b3fc9-Paper.pdf
- Deep DQL https://arxiv.org/pdf/1509.06461
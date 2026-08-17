Here is how to enable a 1M-token context window in Codex for GPT-5.6 Sol. 

Even though we have tuned the context limit in Codex to be set optimally when it comes to performance and cost, this is a common ask, so here it is documented.

A larger context window lets Codex retain more code, tool output, and conversation history before summarizing older material. You need a model that supports it. And GPT-5.6 Sol, for example, has a documented 1,050,000-token window. 

Open ~/.codex/config.toml and add or update these settings at the top level, before any [section] headers:

```
model = "gpt-5.6-sol"
model_context_window = 1000000
model_auto_compact_token_limit = 900000
```

The first setting selects the model. The second tells Codex to use a one-million-token context budget. The third starts automatic history compaction around 900,000 tokens, leaving some headroom. Restart Codex client and start a new session after saving. 

To try the configuration for a single CLI session without changing your defaults:

```
codex -m gpt-5.6-sol \
  -c model_context_window=1000000 \
  -c model_auto_compact_token_limit=900000
```

Have fun, but also know that we tuned the default carefully!
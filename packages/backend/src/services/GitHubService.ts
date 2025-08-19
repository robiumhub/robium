
export interface CreateRepoOptions {
  name: string;
  description?: string;
  private?: boolean;
  autoInit?: boolean;
}

export interface GitHubRepo {
  id: number;
  name: string;
  full_name: string;
  html_url: string;
  owner: { login: string };
}

export class GitHubService {
  private token: string;
  private apiBase: string;

  constructor(token?: string) {
    this.token = token || process.env.GITHUB_TOKEN || '';
    this.apiBase = 'https://api.github.com';
    if (!this.token) {
      throw new Error('GITHUB_TOKEN is required');
    }
  }

  async createRepoForAuthenticatedUser(options: CreateRepoOptions): Promise<GitHubRepo> {
    const resp = await (global as any).fetch(`${this.apiBase}/user/repos`, {
      method: 'POST',
      headers: {
        Authorization: `Bearer ${this.token}`,
        Accept: 'application/vnd.github+json',
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        name: options.name,
        description: options.description || '',
        private: options.private ?? false,
        auto_init: options.autoInit ?? true,
      }),
    });

    if (!resp.ok) {
      const text = await resp.text();
      throw new Error(`GitHub create repo failed: ${resp.status} ${text}`);
    }

    return (await resp.json()) as GitHubRepo;
  }

  async createOrUpdateFiles(owner: string, repo: string, files: { path: string; content: string }[], commitMessage: string): Promise<void> {
    for (const f of files) {
      // Get current SHA if file exists
      const getResp = await (global as any).fetch(
        `${this.apiBase}/repos/${owner}/${repo}/contents/${encodeURIComponent(f.path)}`,
        {
          headers: {
            Authorization: `Bearer ${this.token}`,
            Accept: 'application/vnd.github+json',
          },
        }
      );

      const exists = getResp.status === 200;
      let sha: string | undefined;
      if (exists) {
        const meta = await getResp.json();
        sha = meta.sha;
      }

      const putResp = await (global as any).fetch(
        `${this.apiBase}/repos/${owner}/${repo}/contents/${encodeURIComponent(f.path)}`,
        {
          method: 'PUT',
          headers: {
            Authorization: `Bearer ${this.token}`,
            Accept: 'application/vnd.github+json',
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            message: commitMessage,
            content: Buffer.from(f.content, 'utf-8').toString('base64'),
            sha,
          }),
        }
      );

      if (!putResp.ok) {
        const text = await putResp.text();
        throw new Error(`GitHub commit failed for ${f.path}: ${putResp.status} ${text}`);
      }
    }
  }

  async createRepoFromTemplate(templateOwner: string, templateRepo: string, options: CreateRepoOptions): Promise<GitHubRepo> {
    const resp = await (global as any).fetch(
      `${this.apiBase}/repos/${encodeURIComponent(templateOwner)}/${encodeURIComponent(templateRepo)}/generate`,
      {
        method: 'POST',
        headers: {
          Authorization: `Bearer ${this.token}`,
          Accept: 'application/vnd.github+json',
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          owner: undefined, // authenticated user
          name: options.name,
          description: options.description || '',
          private: options.private ?? false,
        }),
      }
    );

    if (!resp.ok) {
      const text = await resp.text();
      throw new Error(`GitHub template generate failed: ${resp.status} ${text}`);
    }

    return (await resp.json()) as GitHubRepo;
  }

  private async getAuthenticatedUserLogin(): Promise<string> {
    const resp = await (global as any).fetch(`${this.apiBase}/user`, {
      headers: {
        Authorization: `Bearer ${this.token}`,
        Accept: 'application/vnd.github+json',
      },
    });
    if (!resp.ok) {
      const text = await resp.text();
      throw new Error(`GitHub /user failed: ${resp.status} ${text}`);
    }
    const data = await resp.json();
    return data.login as string;
  }

  async forkRepo(templateOwner: string, templateRepo: string, options: { name: string; organization?: string; waitSeconds?: number }): Promise<GitHubRepo> {
    const body: any = { name: options.name };
    if (options.organization) body.organization = options.organization;

    const forkResp = await (global as any).fetch(
      `${this.apiBase}/repos/${encodeURIComponent(templateOwner)}/${encodeURIComponent(templateRepo)}/forks`,
      {
        method: 'POST',
        headers: {
          Authorization: `Bearer ${this.token}`,
          Accept: 'application/vnd.github+json',
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(body),
      }
    );

    if (!forkResp.ok && forkResp.status !== 202) {
      const text = await forkResp.text();
      throw new Error(`GitHub fork failed: ${forkResp.status} ${text}`);
    }

    // Poll until repo is available
    const ownerLogin = options.organization || (await this.getAuthenticatedUserLogin());
    const waitTotal = (options.waitSeconds ?? 20) * 1000;
    const start = Date.now();
    while (Date.now() - start < waitTotal) {
      const getResp = await (global as any).fetch(
        `${this.apiBase}/repos/${encodeURIComponent(ownerLogin)}/${encodeURIComponent(options.name)}`,
        {
          headers: {
            Authorization: `Bearer ${this.token}`,
            Accept: 'application/vnd.github+json',
          },
        }
      );
      if (getResp.ok) {
        return (await getResp.json()) as GitHubRepo;
      }
      await new Promise((r) => setTimeout(r, 2000));
    }
    throw new Error('Timed out waiting for fork to be created');
  }
}

export const getGitHubService = () => new GitHubService();



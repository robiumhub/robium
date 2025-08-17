
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
}

export const getGitHubService = () => new GitHubService();




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
    const resp = await fetch(`${this.apiBase}/user/repos`, {
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
}

export const getGitHubService = () => new GitHubService();



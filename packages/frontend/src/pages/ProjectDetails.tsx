import React, { useEffect, useState } from 'react';
import {
	Box,
	Typography,
	Card,
	CardContent,
	Grid,
	Button,
	Chip,
	Avatar,
	Divider,
	List,
	ListItem,
	ListItemText,
	ListItemAvatar,
} from '@mui/material';
import { useParams, Link as RouterLink } from 'react-router-dom';
import {
	ArrowBack as ArrowBackIcon,
	Edit as EditIcon,
	GitHub as GitHubIcon,
	ContentCopy as ContentCopyIcon,
	Code as CodeIcon,
} from '@mui/icons-material';
import ApiService from '../services/api';

const ProjectDetails: React.FC = () => {
	const { projectId } = useParams<{ projectId: string }>();
	const [project, setProject] = useState<any | null>(null);
	const [loading, setLoading] = useState(true);
	const [error, setError] = useState<string | null>(null);

	useEffect(() => {
		const load = async () => {
			try {
				setLoading(true);
				setError(null);
				const data = await ApiService.get<any>(`/projects/${projectId}`);
				setProject(data);
			} catch (err) {
				setError(err instanceof Error ? err.message : 'Failed to load project');
			} finally {
				setLoading(false);
			}
		};
		if (projectId) load();
	}, [projectId]);

	const getStatusColor = (status: string) => {
		switch (status) {
			case 'online':
				return 'success';
			case 'offline':
				return 'error';
			case 'maintenance':
				return 'warning';
			default:
				return 'default';
		}
	};

	const cloneUrl = project?.github_repo_url ? `${project.github_repo_url}.git` : '';
	const copyClone = async () => {
		try {
			if (cloneUrl) await navigator.clipboard.writeText(`git clone ${cloneUrl}`);
		} catch (_) {}
	};

	if (loading) {
		return (
			<Box sx={{ p: 3 }}>
				<Typography>Loading...</Typography>
			</Box>
		);
	}

	if (error || !project) {
		return (
			<Box sx={{ p: 3 }}>
				<Button component={RouterLink} to="/projects" startIcon={<ArrowBackIcon />} sx={{ mb: 2 }}>
					Back to Projects
				</Button>
				<Typography color="error">{error || 'Project not found'}</Typography>
			</Box>
		);
	}

	return (
		<Box>
			<Box sx={{ display: 'flex', alignItems: 'center', mb: 3 }}>
				<Button component={RouterLink} to="/projects" startIcon={<ArrowBackIcon />} sx={{ mr: 2 }}>
					Back to Projects
				</Button>
				<Typography variant="h4" component="h1" sx={{ flexGrow: 1 }}>
					{project.name}
				</Typography>
				<Button
					variant="contained"
					startIcon={<CodeIcon />}
					component={RouterLink}
					to={`/workspace/${project.id}`}
					sx={{ ml: 1 }}
				>
					Open Workspace
				</Button>
				<Button
					variant="outlined"
					startIcon={<EditIcon />}
					component={RouterLink}
					to={`/projects/${project.id}/edit`}
					sx={{ ml: 1 }}
				>
					Edit Project
				</Button>
			</Box>

			<Grid container spacing={3}>
				<Grid item xs={12} md={8}>
					<Card sx={{ mb: 3 }}>
						<CardContent>
							<Typography variant="h6" gutterBottom>
								Project Overview
							</Typography>
							<Typography variant="body1" color="text.secondary" paragraph>
								{project.description}
							</Typography>
							{project.github_repo_url && (
								<Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 2 }}>
									<Button
										size="small"
										startIcon={<GitHubIcon />}
										href={project.github_repo_url}
										target="_blank"
										rel="noopener noreferrer"
									>
										View Repository
									</Button>
									<Button size="small" startIcon={<ContentCopyIcon />} onClick={copyClone}>
										Copy Clone Command
									</Button>
								</Box>
							)}
							<Box sx={{ display: 'flex', gap: 2, flexWrap: 'wrap' }}>
								<Chip label={`Type: ${project.type || 'custom'}`} />
								<Chip
									label={`Updated: ${new Date(project.updated_at).toLocaleDateString()}`}
								/>
							</Box>
						</CardContent>
					</Card>

					<Card>
						<CardContent>
							<Typography variant="h6" gutterBottom>
								Modules in Project
							</Typography>
							<List>
								{(project.modules || []).map((module: any) => (
									<React.Fragment key={module.id}>
										<ListItem>
											<ListItemAvatar>
												<Avatar>{module.name.charAt(0)}</Avatar>
											</ListItemAvatar>
											<ListItemText
												primary={
													<Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
													<Typography variant="subtitle1">{module.name}</Typography>
												</Box>
											}
											secondary={`Type: ${module.type || ''}`}
										/>
										<Button component={RouterLink} to={`/modules`} size="small">
											View Details
										</Button>
									</ListItem>
									<Divider />
								</React.Fragment>
								))}
							</List>
						</CardContent>
					</Card>
				</Grid>

				<Grid item xs={12} md={4}>
					<Card>
						<CardContent>
							<Typography variant="h6" gutterBottom>
								Quick Actions
							</Typography>
							<Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
								<Button
									variant="contained"
									fullWidth
									component={RouterLink}
									to={`/projects/${projectId}/edit`}
								>
									Edit Project
								</Button>
								<Button
									variant="outlined"
									fullWidth
									component={RouterLink}
									to={`/workspace/${project.id}`}
									startIcon={<CodeIcon />}
								>
									Open Workspace
								</Button>
								{project.github_repo_url && (
									<>
										<Button
											variant="outlined"
											fullWidth
											href={project.github_repo_url}
											target="_blank"
											rel="noopener noreferrer"
											startIcon={<GitHubIcon />}
										>
											See Repository
										</Button>
										<Button
											variant="outlined"
											fullWidth
											onClick={copyClone}
											startIcon={<ContentCopyIcon />}
										>
											Clone to Local (copy)
										</Button>
									</>
								)}
							</Box>
						</CardContent>
					</Card>
				</Grid>
			</Grid>
		</Box>
	);
};

export default ProjectDetails;

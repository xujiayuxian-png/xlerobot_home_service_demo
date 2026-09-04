import type { Task } from './types'

export function upsertTaskHistory(tasks: Task[], task: Task): Task[] {
  return [task, ...tasks.filter(item => item.task_id !== task.task_id)]
    .sort((left, right) => right.updated_at.localeCompare(left.updated_at))
    .slice(0, 20)
}
